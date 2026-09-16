#ifndef BSP_REMOTE_UART_H
#define BSP_REMOTE_UART_H
#include "FreeRTOS.h"
#include "dbus_protocol.h"
#include <stdbool.h>
typedef struct {
    uint8_t data[DBUS_FRAME_BYTES];//18
    TickType_t received_at;
    bool valid_length;
} RemotePacket;
typedef struct {
    uint32_t idle_events, frames, length_errors;
    uint32_t uart_errors, last_uart_error, restart_attempts, start_failures;
} RemoteRxDebug;
extern volatile RemoteRxDebug debug_remote_rx;
/* Init before tasks; service/take only from RemoteTask. */
bool bsp_remote_uart_init(void);
bool bsp_remote_uart_service(void);
bool bsp_remote_uart_take(RemotePacket *packet);
#endif
