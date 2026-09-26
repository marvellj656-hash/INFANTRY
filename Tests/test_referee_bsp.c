#include "bsp_referee.h"
#include "bsp_uart.h"
#include "queue.h"
#include "usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart3;
static DMA_HandleTypeDef dma_handle;
UART_HandleTypeDef huart6 = {.hdmarx = &dma_handle};
static uint8_t *dma;
static unsigned dma_size, position;
static uint32_t tick;
static int event;
static bool fail_start, fail_abort, fail_tx;
static const uint8_t *tx_pointer;
static unsigned tx_length;
static unsigned abort_calls, start_calls;
static uint8_t remote_storage[sizeof(BspUartPacket)];
static StaticQueue_t remote_queue;

QueueHandle_t xQueueCreateStatic(UBaseType_t length, UBaseType_t bytes, uint8_t *storage, StaticQueue_t *object)
{
    *object = (StaticQueue_t){.storage = storage, .length = length, .bytes = bytes};
    return object;
}
QueueHandle_t xQueueCreate(unsigned length, unsigned bytes)
{
    assert(length == 1u && bytes == sizeof(BspUartPacket));
    return xQueueCreateStatic(length, bytes, remote_storage, &remote_queue);
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait)
{
    assert(wait == 0u);
    if (!queue->count) return pdFALSE;
    memcpy(out, queue->storage + queue->head * queue->bytes, queue->bytes);
    queue->head = (queue->head + 1u) % queue->length;
    --queue->count;
    return pdPASS;
}
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *data, BaseType_t *wake)
{
    *wake = pdFALSE;
    if (queue->count == queue->length) return pdFALSE;
    memcpy(queue->storage + queue->tail * queue->bytes, data, queue->bytes);
    queue->tail = (queue->tail + 1u) % queue->length;
    ++queue->count;
    return pdPASS;
}
BaseType_t xQueueOverwriteFromISR(QueueHandle_t queue, const void *data, BaseType_t *wake)
{
    assert(queue->length == 1u);
    (void)xQueueReset(queue);
    return xQueueSendFromISR(queue, data, wake);
}
BaseType_t xQueueReset(QueueHandle_t queue)
{
    queue->head = queue->tail = queue->count = 0u;
    return pdPASS;
}
uint32_t bsp_time_ms(void) { return tick; }
uint32_t bsp_time_ms_from_isr(void) { return tick; }
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *h)
{
    assert(h == &huart6);
    ++abort_calls;
    return fail_abort ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h)
{
    assert(h == &huart3);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *b, uint16_t n)
{
    assert(h == &huart6);
    dma = b; dma_size = n; position = 0u; dma_handle.counter = n;
    ++start_calls;
    return fail_start ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *h, const uint8_t *b, uint16_t n)
{
    assert(h == &huart6);
    tx_pointer = b; tx_length = n;
    return fail_tx ? HAL_BUSY : HAL_OK;
}
int HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *h) { (void)h; return event; }

static void feed(const uint8_t *bytes, unsigned length, bool idle, bool notify_dma)
{
    for (unsigned i = 0; i < length; ++i) {
        dma[position++] = bytes[i];
        dma_handle.counter = dma_size - position;
        if (position == dma_size / 2u || position == dma_size) {
            if (notify_dma) {
                event = position == dma_size ? HAL_UART_RXEVENT_TC : HAL_UART_RXEVENT_HT;
                HAL_UARTEx_RxEventCallback(&huart6, (uint16_t)position);
            }
            if (position == dma_size) { position = 0u; dma_handle.counter = dma_size; }
        }
    }
    if (idle) {
        event = HAL_UART_RXEVENT_IDLE;
        HAL_UARTEx_RxEventCallback(&huart6, (uint16_t)(position ? position : dma_size));
    }
}

static void expect_bytes(const uint8_t *bytes, unsigned length, uint32_t timestamp)
{
    BspRefereePacket packet;
    unsigned offset = 0;
    while (bsp_referee_receive(&packet)) {
        assert(packet.length > 0u && packet.length <= 128u);
        assert(packet.received_at_ms == timestamp);
        assert(offset + packet.length <= length);
        assert(memcmp(packet.data, bytes + offset, packet.length) == 0);
        offset += packet.length;
    }
    assert(offset == length);
}

int main(void)
{
    assert(bsp_referee_start());
    assert(dma_size == 1024u && start_calls == 1u && abort_calls == 1u);
    uint32_t generation = bsp_referee_generation();
    assert(bsp_referee_start() && start_calls == 1u);
    uint8_t bytes[1024];
    for (unsigned i = 0; i < sizeof(bytes); ++i) bytes[i] = (uint8_t)i;

    /* 连续完整环由 HT/TC 分两次取走，IDLE重复通知不得重复交包。 */
    tick = 7u;
    feed(bytes, sizeof(bytes), true, true);
    expect_bytes(bytes, sizeof(bytes), tick);
    feed(bytes, 0u, true, true);
    expect_bytes(bytes, 0u, tick);
    /* 从任意位置跨512与1024边界，块顺序与原字节顺序相同。 */
    for (unsigned i = 0; i < 12u; ++i) {
        tick += 10u;
        feed(bytes, 317u, true, true);
        expect_bytes(bytes, 317u, tick);
    }

    /* 先处理IDLE再处理迟到HT：读取真实NDTR不会重复入队旧字节。 */
    huart6.ErrorCode = 8u; HAL_UART_ErrorCallback(&huart6);
    assert(bsp_referee_start());
    tick = 200u;
    feed(bytes, 600u, true, false);
    expect_bytes(bytes, 600u, tick);
    event = HAL_UART_RXEVENT_HT;
    HAL_UARTEx_RxEventCallback(&huart6, 512u);
    expect_bytes(bytes, 0u, tick);
    /* 先IDLE跨回绕，随后迟到TC，同样不能重新发送旧环。 */
    feed(bytes, 500u, true, false);
    expect_bytes(bytes, 500u, tick);
    event = HAL_UART_RXEVENT_TC;
    HAL_UARTEx_RxEventCallback(&huart6, 1024u);
    expect_bytes(bytes, 0u, tick);

    /* FIFO满不是丢一块后继续拼包：立即隔离整条旧字节流。 */
    generation = bsp_referee_generation();
    for (unsigned i = 0; i < 18u; ++i) feed(bytes, 128u, true, true);
    BspRefereePacket packet;
    assert(debug_referee_rx.queue_overflows == 1u);
    assert(bsp_referee_generation() != generation);
    assert(!bsp_referee_receive(&packet));
    uint32_t packets = debug_referee_rx.rx_packets;
    feed(bytes, 128u, true, true);
    assert(debug_referee_rx.rx_packets == packets);
    assert(bsp_referee_start());
    assert(!bsp_referee_receive(&packet));
    feed(bytes, 30u, true, true);
    expect_bytes(bytes, 30u, tick);

    /* UART故障、Abort失败、重启失败都不能放行旧数据。 */
    feed(bytes, 20u, true, true);
    generation = bsp_referee_generation();
    huart6.ErrorCode = 4u; HAL_UART_ErrorCallback(&huart6);
    assert(bsp_referee_generation() != generation && !bsp_referee_receive(&packet));
    fail_abort = true;
    assert(!bsp_referee_start());
    fail_abort = false; fail_start = true;
    assert(!bsp_referee_start());
    fail_start = false;
    assert(bsp_referee_start() && !bsp_referee_receive(&packet));
    feed(bytes, 20u, true, true);
    expect_bytes(bytes, 20u, tick);

    /* 发送复制到常驻内存：用户栈数组改了，正在发的字节不变。 */
    uint8_t tx[] = {1u, 2u, 3u, 4u};
    assert(bsp_referee_send(tx, sizeof(tx)));
    assert(tx_pointer != tx && tx_length == sizeof(tx));
    tx[0] = 9u;
    assert(tx_pointer[0] == 1u && !bsp_referee_send(tx, sizeof(tx)));
    UART_HandleTypeDef other = {0};
    HAL_UART_TxCpltCallback(&other);
    assert(!bsp_referee_send(tx, sizeof(tx)));
    HAL_UART_TxCpltCallback(&huart6);
    assert(bsp_referee_send(tx, sizeof(tx)));
    /* 错误恢复负责终止旧TX并清busy，随后可以再次发送。 */
    HAL_UART_ErrorCallback(&huart6);
    assert(!bsp_referee_send(tx, sizeof(tx)));
    assert(bsp_referee_start() && bsp_referee_send(tx, sizeof(tx)));
    HAL_UART_TxCpltCallback(&huart6);
    fail_tx = true;
    assert(!bsp_referee_send(tx, sizeof(tx)));
    fail_tx = false;
    assert(bsp_referee_start() && bsp_referee_send(tx, sizeof(tx)));
    /* 完成中断丢失时100ms后也能恢复，避免永远busy。 */
    tick += 100u;
    assert(bsp_referee_start() && debug_referee_rx.tx_timeouts == 1u);
    assert(bsp_referee_send(tx, sizeof(tx)));
    HAL_UART_TxCpltCallback(&huart6);
    assert(!bsp_referee_send(NULL, 1u) && !bsp_referee_send(tx, 0u));
    assert(!bsp_referee_send(bytes, 129u));

    /* 强分发钩子只截获USART6，USART3原错误回调仍工作。 */
    unsigned remote_errors = debug_remote_rx.uart_errors;
    huart3.ErrorCode = 2u; HAL_UART_ErrorCallback(&huart3);
    assert(debug_remote_rx.uart_errors == remote_errors + 1u);
    unsigned referee_errors = debug_referee_rx.uart_errors;
    HAL_UART_ErrorCallback(&other);
    assert(debug_referee_rx.uart_errors == referee_errors);
    puts("referee BSP: DMA boundaries, late callbacks, FIFO recovery, persistent TX PASS");
    return 0;
}
