#include "bsp_nuc.h"
#include "bsp_time.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>

extern USBD_DescriptorsTypeDef nuc_usb_descriptors;
static USBD_HandleTypeDef usb_device;
static USBD_CDC_HandleTypeDef cdc_memory;
static bool cdc_allocated, started;
static volatile uint32_t generation;
static QueueHandle_t rx_queue;
/* USB FS 每包最多 64 字节。都在普通 SRAM，发完之前不会失效。 */
static uint8_t rx_bytes[BSP_NUC_PACKET_BYTES] __attribute__((aligned(4)));
static uint8_t tx_bytes[BSP_NUC_PACKET_BYTES] __attribute__((aligned(4)));
static uint8_t line_coding[7] = {0x00, 0xc2, 0x01, 0x00, 0, 0, 8}; /* 115200,8N1 */
typedef struct
{
    BspNucPacket packet;
    uint32_t generation;
} QueuedNucPacket;
volatile BspNucDebug debug_nuc_usb;

void *bsp_nuc_usb_alloc(size_t size)
{
    if (size > sizeof(cdc_memory) || cdc_allocated)
        return NULL;
    cdc_allocated = true;
    memset(&cdc_memory, 0, sizeof(cdc_memory));
    return &cdc_memory;
}
void bsp_nuc_usb_free(void *memory)
{
    if (memory == &cdc_memory)
        cdc_allocated = false;
}
void bsp_nuc_link_changed(void)
{
    generation++;
    debug_nuc_usb.generation = generation;
    debug_nuc_usb.resets++;
    debug_nuc_usb.configured = 0u;
}
uint32_t bsp_nuc_generation(void) { return generation; }

static int8_t cdc_init(void)
{
    bsp_nuc_link_changed();
    USBD_CDC_SetTxBuffer(&usb_device, tx_bytes, 0u);
    USBD_CDC_SetRxBuffer(&usb_device, rx_bytes);
    debug_nuc_usb.configured = 1u;
    return USBD_OK;
}
static int8_t cdc_deinit(void)
{
    bsp_nuc_link_changed();
    return USBD_OK;
}
static int8_t cdc_control(uint8_t command, uint8_t *buffer, uint16_t length)
{
    /* USB 的波特率设置只是“虚拟串口配置”，不会拿去重新配置任何 UART。 */
    if (!buffer)
        return USBD_FAIL;
    if (command == CDC_SET_LINE_CODING && length == sizeof(line_coding))
        memcpy(line_coding, buffer, sizeof(line_coding));
    else if (command == CDC_GET_LINE_CODING && length == sizeof(line_coding))
        memcpy(buffer, line_coding, sizeof(line_coding));
    return USBD_OK;
}
static int8_t cdc_receive(uint8_t *buffer, uint32_t *length)
{
    BaseType_t wake = pdFALSE;
    if (rx_queue && buffer && length && *length && *length <= BSP_NUC_PACKET_BYTES)
    {
        QueuedNucPacket entry = {.generation = generation};
        entry.packet.length = (uint16_t)*length;
        entry.packet.received_at_ms = bsp_time_ms_from_isr();
        memcpy(entry.packet.data, buffer, *length);
        if (xQueueSendFromISR(rx_queue, &entry, &wake) == pdPASS)
        {
            debug_nuc_usb.rx_packets++;
            debug_nuc_usb.rx_bytes += *length;
        }
        else
        {
            /* 丢一个 USB 块会打断半帧。换代后队列里的旧块也不再参与拼帧。 */
            debug_nuc_usb.dropped_packets++;
            bsp_nuc_link_changed();
        }
    }
    USBD_CDC_SetRxBuffer(&usb_device, rx_bytes);
    USBD_CDC_ReceivePacket(&usb_device);
    portYIELD_FROM_ISR(wake);
    return USBD_OK;
}
static int8_t cdc_transmit_complete(uint8_t *buffer, uint32_t *length, uint8_t ep)
{
    (void)buffer;
    (void)length;
    (void)ep;
    return USBD_OK;
}
static USBD_CDC_ItfTypeDef cdc_interface = {
    cdc_init, cdc_deinit, cdc_control, cdc_receive, cdc_transmit_complete};

bool bsp_nuc_start(void)
{
    if (started)
        return true;
    if (!rx_queue)
        rx_queue = xQueueCreate(16u, sizeof(QueuedNucPacket));
    if (!rx_queue)
        return false;
    /* 这里不要 taskENTER_CRITICAL：PCD 初始化需要 HAL 毫秒时钟继续走。 */
    if (USBD_Init(&usb_device, &nuc_usb_descriptors, 0u) != USBD_OK ||
        USBD_RegisterClass(&usb_device, USBD_CDC_CLASS) != USBD_OK ||
        USBD_CDC_RegisterInterface(&usb_device, &cdc_interface) != USBD_OK ||
        USBD_Start(&usb_device) != USBD_OK)
    {
        USBD_DeInit(&usb_device);
        debug_nuc_usb.init_failures++;
        return false;
    }
    started = true;
    debug_nuc_usb.initialized = 1u;
    return true;
}
bool bsp_nuc_ready(void)
{
    bool ready = started && usb_device.dev_state == USBD_STATE_CONFIGURED &&
                 usb_device.pClassData != NULL;
    debug_nuc_usb.configured = ready;
    return ready;
}
bool bsp_nuc_receive(BspNucPacket *out)
{
    if (!out || !rx_queue)
        return false;
    QueuedNucPacket entry;
    while (xQueueReceive(rx_queue, &entry, 0u) == pdPASS)
    {
        if (entry.generation == generation)
        {
            *out = entry.packet;
            return true;
        }
    }
    return false;
}
bool bsp_nuc_send(const uint8_t *data, size_t length)
{
    if (!data || !length || length > sizeof(tx_bytes))
        return false;
    bool accepted = false;
    /* 只保护“看 busy + 复制 + 交给 USB”这几步，中断不会在中途释放 CDC 对象。 */
    taskENTER_CRITICAL();
    if (bsp_nuc_ready())
    {
        USBD_CDC_HandleTypeDef *cdc = usb_device.pClassData;
        if (cdc->TxState == 0u)
        {
            memcpy(tx_bytes, data, length);
            USBD_CDC_SetTxBuffer(&usb_device, tx_bytes, (uint32_t)length);
            accepted = USBD_CDC_TransmitPacket(&usb_device) == USBD_OK;
        }
    }
    taskEXIT_CRITICAL();
    if (accepted)
        debug_nuc_usb.tx_packets++;
    else
        debug_nuc_usb.tx_busy++;
    return accepted;
}
