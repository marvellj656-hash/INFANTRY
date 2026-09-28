#include "bsp_nuc.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "USB check line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

USBD_DescriptorsTypeDef nuc_usb_descriptors;
USBD_ClassTypeDef USBD_CDC;
static uint32_t now_ms, tx_length, receive_rearms;
static uint8_t *tx_pointer;
static USBD_HandleTypeDef *device;
static USBD_CDC_ItfTypeDef *interface;
static bool init_fails = true;
static struct { uint8_t data[16][128]; unsigned size, length, used, head; } queue;
uint32_t bsp_time_ms_from_isr(void) { return now_ms; }
QueueHandle_t xQueueCreate(unsigned length, unsigned size)
{
    CHECK(length <= 16u && size <= 128u);
    queue.length = length;
    queue.size = size;
    queue.used = queue.head = 0u;
    return &queue;
}
BaseType_t xQueueSendFromISR(QueueHandle_t q, const void *in, BaseType_t *wake)
{
    (void)wake;
    CHECK(q == &queue);
    if (queue.used == queue.length) return pdFALSE;
    memcpy(queue.data[(queue.head + queue.used) % queue.length], in, queue.size);
    queue.used++;
    return pdPASS;
}
BaseType_t xQueueReceive(QueueHandle_t q, void *out, TickType_t wait)
{
    CHECK(q == &queue && wait == 0u);
    if (!queue.used) return pdFALSE;
    memcpy(out, queue.data[queue.head], queue.size);
    queue.head = (queue.head + 1u) % queue.length;
    queue.used--;
    return pdPASS;
}
uint8_t USBD_Init(USBD_HandleTypeDef *d, USBD_DescriptorsTypeDef *desc, uint8_t id)
{
    (void)desc; (void)id;
    device = d;
    return init_fails ? USBD_FAIL : USBD_OK;
}
uint8_t USBD_DeInit(USBD_HandleTypeDef *d)
{
    if (d->pClassData) { interface->DeInit(); bsp_nuc_usb_free(d->pClassData); }
    d->pClassData = NULL;
    d->dev_state = 0u;
    return USBD_OK;
}
uint8_t USBD_RegisterClass(USBD_HandleTypeDef *d, USBD_ClassTypeDef *c)
{
    CHECK(d == device && c == &USBD_CDC); return USBD_OK;
}
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *d, USBD_CDC_ItfTypeDef *itf)
{
    CHECK(d == device); interface = itf; return USBD_OK;
}
uint8_t USBD_Start(USBD_HandleTypeDef *d)
{
    d->pClassData = bsp_nuc_usb_alloc(sizeof(USBD_CDC_HandleTypeDef));
    CHECK(d->pClassData);
    d->dev_state = USBD_STATE_CONFIGURED;
    return (uint8_t)interface->Init();
}
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *d, uint8_t *data, uint32_t length)
{
    CHECK(d == device); tx_pointer = data; tx_length = length; return USBD_OK;
}
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *d, uint8_t *data)
{
    CHECK(d == device && data); return USBD_OK;
}
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *d)
{
    ((USBD_CDC_HandleTypeDef *)d->pClassData)->TxState = 1u; return USBD_OK;
}
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *d)
{
    CHECK(d == device); receive_rearms++; return USBD_OK;
}
static void receive(uint8_t value)
{
    uint8_t incoming[4] = {value, 2, 3, 4};
    uint32_t length = sizeof(incoming);
    CHECK(interface->Receive(incoming, &length) == USBD_OK);
    memset(incoming, 0, sizeof(incoming));
}
int main(void)
{
    CHECK(!bsp_nuc_start());
    CHECK(debug_nuc_usb.init_failures == 1u);
    init_fails = false;
    CHECK(bsp_nuc_start() && bsp_nuc_ready());
    CHECK(bsp_nuc_start());
    CHECK(!bsp_nuc_usb_alloc(sizeof(USBD_CDC_HandleTypeDef))); /* 只分配一个CDC对象 */
    uint8_t data[] = {0xff, 1, 2, 3};
    CHECK(bsp_nuc_send(data, sizeof(data)));
    CHECK(debug_nuc_usb.tx_packets == 1u && debug_nuc_usb.tx_completed == 0u);
    CHECK(interface->TransmitCplt(tx_pointer, &tx_length, 1u) == USBD_OK);
    CHECK(debug_nuc_usb.tx_completed == 1u && debug_nuc_usb.tx_completed_bytes == 4u);
    data[0] = 0;
    CHECK(tx_length == 4u && tx_pointer[0] == 0xff && tx_pointer != data);
    CHECK(!bsp_nuc_send(data, sizeof(data))); /* busy时不能覆盖未发完的buffer */
    CHECK(tx_pointer[0] == 0xff);
    ((USBD_CDC_HandleTypeDef *)device->pClassData)->TxState = 0;
    CHECK(bsp_nuc_send(data, sizeof(data)) && tx_pointer[0] == 0);
    CHECK(!bsp_nuc_send(data, 65u) && !bsp_nuc_send(NULL, 4u));

    BspNucPacket packet;
    now_ms = 123;
    receive(9);
    CHECK(bsp_nuc_receive(&packet));
    CHECK(packet.data[0] == 9 && packet.length == 4u && packet.received_at_ms == 123u);
    CHECK(!bsp_nuc_receive(&packet));
    uint32_t generation = bsp_nuc_generation();
    for (unsigned i = 0; i < 17u; ++i) receive((uint8_t)i);
    CHECK(debug_nuc_usb.dropped_packets == 1u && bsp_nuc_generation() != generation);
    CHECK(!bsp_nuc_receive(&packet)); /* 溢出前排队的所有半包都作废 */
    receive(77);
    CHECK(bsp_nuc_receive(&packet) && packet.data[0] == 77);
    receive(78);
    bsp_nuc_link_changed();
    CHECK(!bsp_nuc_receive(&packet));

    CHECK(USBD_DeInit(device) == USBD_OK && !bsp_nuc_ready());
    CHECK(!bsp_nuc_send(data, sizeof(data)));
    CHECK(USBD_Start(device) == USBD_OK && bsp_nuc_ready());
    receive(88);
    CHECK(bsp_nuc_receive(&packet) && packet.data[0] == 88);
    CHECK(receive_rearms == 21u);
    puts("nuc_usb: init retry, static memory, TX copy/busy, queue overflow and reconnect passed");
    return 0;
}
