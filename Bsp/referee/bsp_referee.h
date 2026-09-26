#ifndef BSP_REFEREE_H
#define BSP_REFEREE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BSP_REFEREE_PACKET_BYTES 128u
typedef struct {
    uint8_t data[BSP_REFEREE_PACKET_BYTES];
    uint16_t length;
    uint32_t received_at_ms;
} BspRefereePacket;

typedef struct {
    uint32_t rx_bytes;
    uint32_t rx_packets;
    uint32_t idle_events;
    uint32_t queue_overflows;
    uint32_t uart_errors;
    uint32_t last_uart_error;
    uint32_t invalid_dma_events;
    uint32_t restart_attempts;
    uint32_t start_failures;
    uint32_t tx_started;
    uint32_t tx_completed;
    uint32_t tx_timeouts;
    uint32_t tx_failures;
} RefereeRxDebug;
extern volatile RefereeRxDebug debug_referee_rx;

/* 任务调用：首次创建静态队列并启动；之后重复调用仅在故障时恢复。 */
bool bsp_referee_start(void);
bool bsp_referee_receive(BspRefereePacket *out);
/* 数据先复制进本模块常驻缓冲区；返回后调用者可以立刻改/释放原数组。 */
bool bsp_referee_send(const uint8_t *data, size_t length);
/* ISR 检测到丢数据即改变代次，接收任务据此丢掉协议层半包。 */
uint32_t bsp_referee_generation(void);

#endif
