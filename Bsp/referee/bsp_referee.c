#include "bsp_referee.h"
#include "bsp_time.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>

#define REFEREE_DMA_BYTES 1024u
#define REFEREE_QUEUE_LENGTH 16u
#define REFEREE_TX_TIMEOUT_MS 100u

/* DMA 只访问普通 SRAM，不能放进 STM32F407 的 CCM RAM。 */
static uint8_t dma_buffer[REFEREE_DMA_BYTES];
static uint8_t tx_buffer[BSP_REFEREE_PACKET_BYTES];
static uint16_t cursor;
static QueueHandle_t rx_queue;
static StaticQueue_t rx_queue_object;
static uint8_t rx_queue_storage[REFEREE_QUEUE_LENGTH * sizeof(BspRefereePacket)];
static volatile bool restart_needed = true;
static volatile bool tx_busy;
static uint32_t tx_started_at_ms;
static volatile uint32_t generation;
volatile RefereeRxDebug debug_referee_rx;

uint32_t bsp_referee_generation(void) { return generation; }

/* 调用环境已经阻止相关中断嵌套：ISR同优先级，任务用短临界区。 */
static void invalidate_stream(void)
{
    if (!restart_needed) ++generation;
    restart_needed = true;
}

bool bsp_referee_start(void)
{
    if (!rx_queue) {
        rx_queue = xQueueCreateStatic(REFEREE_QUEUE_LENGTH, sizeof(BspRefereePacket),
                                      rx_queue_storage, &rx_queue_object);
        if (!rx_queue) return false;
    }
    taskENTER_CRITICAL();
    uint32_t now = bsp_time_ms();
    if (!restart_needed && tx_busy && (uint32_t)(now - tx_started_at_ms) >= REFEREE_TX_TIMEOUT_MS) {
        ++debug_referee_rx.tx_timeouts;
        invalidate_stream();
    }
    bool needs_restart = restart_needed;
    taskEXIT_CRITICAL();
    if (!needs_restart) return true;

    ++debug_referee_rx.restart_attempts;
    /* Abort 在任务中做，不让中断等待硬件。一起停止 TX，避免错误后 busy 永远挂住。 */
    if (HAL_UART_Abort(&huart6) != HAL_OK) {
        ++debug_referee_rx.start_failures;
        return false;
    }
    taskENTER_CRITICAL();
    ++generation;
    cursor = 0;
    tx_busy = false;
    (void)xQueueReset(rx_queue);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    restart_needed = false;
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6, dma_buffer, REFEREE_DMA_BYTES) != HAL_OK) {
        restart_needed = true;
        ++debug_referee_rx.start_failures;
    }
    bool started = !restart_needed;
    taskEXIT_CRITICAL();
    return started;
}

bool bsp_referee_receive(BspRefereePacket *out)
{
    if (!out || !rx_queue) return false;
    taskENTER_CRITICAL();
    /* 故障已发生时，队列里即使有旧数据，也不能再交给协议层。 */
    bool received = !restart_needed && xQueueReceive(rx_queue, out, 0) == pdPASS;
    taskEXIT_CRITICAL();
    return received;
}

bool bsp_referee_send(const uint8_t *data, size_t length)
{
    if (!data || length == 0u || length > sizeof(tx_buffer)) return false;
    taskENTER_CRITICAL();
    if (restart_needed || tx_busy) {
        taskEXIT_CRITICAL();
        return false;
    }
    memcpy(tx_buffer, data, length);
    tx_busy = true;
    tx_started_at_ms = bsp_time_ms();
    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(&huart6, tx_buffer, (uint16_t)length);
    if (status == HAL_OK) ++debug_referee_rx.tx_started;
    else {
        tx_busy = false;
        ++debug_referee_rx.tx_failures;
        /* HAL_BUSY 也可能是 HAL 状态未恢复，交给下次 start 统一修复。 */
        invalidate_stream();
    }
    taskEXIT_CRITICAL();
    return status == HAL_OK;
}

/* 由 bsp_uart.c 的统一 HAL 回调分发过来。
 * USART6 与 DMA2_Stream1 必须同为可调用 FreeRTOS FromISR 的优先级（本工程为5）。 */
bool bsp_uart_other_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != &huart6) return false;
    if (restart_needed || !rx_queue) return true;
    if (size > REFEREE_DMA_BYTES || !huart->hdmarx) {
        ++debug_referee_rx.invalid_dma_events;
        invalidate_stream();
        return true;
    }
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE)
        ++debug_referee_rx.idle_events;

    /* HT/TC 是“到过这里”的通知，不一定是此刻位置。
     * 例如 IDLE 先处理到600，随后迟到HT通知512；用512会重复拷贝旧字节。
     * 每次都读 DMA 剩余计数，得到此刻真实写入点，重复回调自然变成空操作。 */
    uint32_t remaining = __HAL_DMA_GET_COUNTER(huart->hdmarx);
    if (remaining > REFEREE_DMA_BYTES) {
        ++debug_referee_rx.invalid_dma_events;
        invalidate_stream();
        return true;
    }
    uint16_t end = (uint16_t)((REFEREE_DMA_BYTES - remaining) % REFEREE_DMA_BYTES);
    uint32_t timestamp = bsp_time_ms_from_isr();
    BaseType_t wake = pdFALSE;
    while (cursor != end) {
        BspRefereePacket packet;
        packet.received_at_ms = timestamp;
        uint16_t available = end > cursor ? (uint16_t)(end - cursor) : (uint16_t)(REFEREE_DMA_BYTES - cursor);
        packet.length = available > BSP_REFEREE_PACKET_BYTES ? BSP_REFEREE_PACKET_BYTES : available;
        memcpy(packet.data, dma_buffer + cursor, packet.length);
        if (xQueueSendFromISR(rx_queue, &packet, &wake) != pdPASS) {
            /* 丢一个块后，前后字节就接不上。停止收新块，任务再清队列重开。 */
            ++debug_referee_rx.queue_overflows;
            invalidate_stream();
            break;
        }
        debug_referee_rx.rx_bytes += packet.length;
        ++debug_referee_rx.rx_packets;
        cursor = (uint16_t)((cursor + packet.length) % REFEREE_DMA_BYTES);
    }
    portYIELD_FROM_ISR(wake);
    return true;
}

bool bsp_uart_other_error(UART_HandleTypeDef *huart)
{
    if (huart != &huart6) return false;
    ++debug_referee_rx.uart_errors;
    debug_referee_rx.last_uart_error = huart->ErrorCode;
    invalidate_stream();
    return true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart6) return;
    tx_busy = false;
    ++debug_referee_rx.tx_completed;
}
