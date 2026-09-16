#include "bsp_remote_uart.h"
#include "queue.h"
#include "task.h"
#include "usart.h"
#include <string.h>
#define DMA_BYTES 72u
/* Normal SRAM (.bss), never CCM: DMA cannot access CCM on STM32F407. */
static uint8_t dma_buffer[DMA_BYTES];//72
static uint16_t cursor, frame_length;//cursor :current loc    frame_len: current frame len
static uint8_t frame[DBUS_FRAME_BYTES];//18 RT memory array
static QueueHandle_t packet_queue;// global null
static volatile bool restart_needed = true;
volatile RemoteRxDebug debug_remote_rx;

bool bsp_remote_uart_init(void)//create a queue for one realtime(1) frame non receive to idle
{
    packet_queue = xQueueCreate(1, sizeof(RemotePacket));//overcreate  xqueuereceive
    return packet_queue != NULL;
}
//UART 接收维护/自恢复函数  调用时期:RemoteTask
//放在 RemoteTask 任务上下文执行，不要在 UART 中断/回调函数里做这种比较重的恢复操作
bool bsp_remote_uart_service(void)
{
    if (!restart_needed) return true;//故障标准位
    debug_remote_rx.restart_attempts++;
    /* Abort is performed in task context, never inside the callback. */
    if (HAL_UART_AbortReceive(&huart3) != HAL_OK) return false;//出问题了，停掉DMA,后面重新idle dma
    taskENTER_CRITICAL();//临界区
    cursor = 0; frame_length = 0;//clear state
    xQueueReset(packet_queue);//reset queue
    __HAL_UART_CLEAR_OREFLAG(&huart3);//clear :overflow error
    restart_needed = false;//clear flag
    HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dma_buffer, DMA_BYTES);//修复成功data 进缓冲区，拿到data,true-> 只开启一次
    if (status != HAL_OK) {
        restart_needed = true;
        debug_remote_rx.start_failures++;
    }
    taskEXIT_CRITICAL();//退临界区
    return !restart_needed;
}

bool bsp_remote_uart_take(RemotePacket *packet)//from packet_queue(realtime frame) to &packet
{
    return xQueueReceive(packet_queue, packet, 0) == pdPASS;//TODO:waitfor 0!
}

/* Sole HAL RX-event callback. DMA HT/TC collect bytes; IDLE closes a frame.
 * USART3 and DMA1_Stream1 share priority 5, so these callbacks cannot nest.(嵌套）
 */
//中断回调UART Receive-to-IDLE
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)//size current DMA LOC
{
    if (huart != &huart3 || restart_needed || !packet_queue) return;//UART3  RECOVER INIT
    if (size > DMA_BYTES) { restart_needed = true; return; }
    //entry bsp_remote_uart_service()
    //find abnormal set flag instead of fixing in interrupt
    uint16_t end = (size == DMA_BYTES) ? 0 : size;//circle
    while (cursor != end) {//cursor: head dma buffer's loc   end: dma tail loc
        if (frame_length < DBUS_FRAME_BYTES)
            frame[frame_length] = dma_buffer[cursor];
        /* Saturate at 19      so oversized bursts are rejected, not truncated. */
        if (frame_length <= DBUS_FRAME_BYTES) frame_length++;
        cursor = (cursor + 1u) % DMA_BYTES;//circle buffer index 63+1 ->0 circle
    }//cursor= end indicates ok
    if (HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_IDLE) return;// half transfer or complete is exit
//TODO: just for debug
    debug_remote_rx.idle_events++;
    //idle but no sending exiting
    if (frame_length == 0) return; /* Duplicate boundary notification. */


    //true datapackge
    RemotePacket packet = {0};
    packet.received_at = xTaskGetTickCountFromISR();//in the interrupt context
    packet.valid_length = (frame_length == DBUS_FRAME_BYTES);//to valid the len of package
    if (packet.valid_length) {
        memcpy(packet.data, frame, DBUS_FRAME_BYTES);
        debug_remote_rx.frames++;//for debugging
    } else {
        debug_remote_rx.length_errors++;
    }
    frame_length = 0;
    BaseType_t wake = pdFALSE;
    xQueueOverwriteFromISR(packet_queue, &packet, &wake);//overwrite in queue if something is wating(empty)(blocked) ->ready wake is a flag which is brought to pdTRUE when receive the queue advances
    portYIELD_FROM_ISR(wake);//if pdTRUE ->     redipatching compare priority
    //the result is that receiving queue can be advanced
}// to distinct :suspend    vTaskresumeFromISR()  ->ready
//UART 出现错误时，HAL 会自动调用它 中断
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart3) return;//只关心 USART3
    debug_remote_rx.uart_errors++;
    debug_remote_rx.last_uart_error = huart->ErrorCode;
    restart_needed = true;//任务线程之后去重启 UART/DMA
}
