#ifndef BSP_BSP_UART_H
#define BSP_BSP_UART_H

#include <stdbool.h>
#include <stdint.h>
/* 当前仅实现 USART3 接收。IDLE 表示一次空闲边界，不代表已得到合法 DBUS 帧。 */
#define BSP_UART_PACKET_BYTES 64u
typedef struct {
    uint8_t data[BSP_UART_PACKET_BYTES];
    uint16_t length; /* 超过容量时为 capacity+1，上层必须拒绝，不能截成有效帧。 */
    uint32_t received_at_ms;
} BspUartPacket;
typedef struct {
    uint32_t idle_events, frames, length_errors;
    uint32_t uart_errors, last_uart_error, restart_attempts, start_failures;
} UartRxDebug;
extern volatile UartRxDebug debug_remote_rx; /* 保留 LinkScope 符号。 */
bool bsp_uart_init(void); /* 只创建接收资源，不创建任务。 */
/* 任务上下文：首次启动或故障恢复。generation 改变后，上层丢弃旧解码状态。 */
bool bsp_uart_recover_rx(void);
uint32_t bsp_uart_rx_generation(void);
bool bsp_uart_receive(BspUartPacket *packet); /* 不等待，有数据则复制并取走。 */

#endif
