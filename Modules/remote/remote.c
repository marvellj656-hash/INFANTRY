#include "remote.h"
#include "bsp_uart.h"
#include "bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
/* cmd_task 唯一更新，其他任务读取这份完整快照，不竞争 UART 接收队列。 */
static RemoteState published;
static uint32_t published_timeout_ms;
static DbusState decoded;
static bool valid;
static uint32_t received_at_ms;
volatile RemoteDebug debug_remote;
bool remote_init(void)
{
    published = (RemoteState){.age_ms = UINT32_MAX};
    published_timeout_ms = 0;
    valid = false;
    received_at_ms = 0;
    memset(&decoded, 0, sizeof(decoded));
    return bsp_uart_init();
}
void remote_update(uint32_t timeout_ms, RemoteState *out)
{
    uint32_t generation = bsp_uart_rx_generation();
    bool transport_ok = bsp_uart_recover_rx();
    /* 驱动恢复成功也不能沿用恢复前的数据，必须收到新的有效报文。 */
    if (!transport_ok || bsp_uart_rx_generation() != generation) valid = false;
    BspUartPacket packet;
    if (bsp_uart_receive(&packet)) {
        valid = dbus_decode(packet.data, packet.length, &decoded);
        if (valid) {
            received_at_ms = packet.received_at_ms;
            debug_remote.valid_frames++;
        } else debug_remote.invalid_frames++;
    }
    /* 取包之后再取当前时间，避免刚到达的中断时间戳比 now 更新。 */
    uint32_t age = bsp_time_ms() - received_at_ms;
    bool online = transport_ok && valid && age < timeout_ms;
    *out = (RemoteState){.channels = decoded, .received_at_ms = received_at_ms,
        .age_ms = valid ? age : UINT32_MAX, .online = online};
    debug_remote.ch1 = online ? decoded.ch[0] : 0;
    debug_remote.ch2 = online ? decoded.ch[1] : 0;
    debug_remote.ch3 = online ? decoded.ch[2] : 0;
    debug_remote.ch4 = online ? decoded.ch[3] : 0;
    debug_remote.sw1 = online ? decoded.sw1 : 1;
    debug_remote.sw2 = online ? decoded.sw2 : 1;
    debug_remote.online = online;
    debug_remote.age_ms = out->age_ms;
    /* 只保护复制，UART 恢复和解码不放进临界区。 */
    taskENTER_CRITICAL();
    published = *out;
    published_timeout_ms = timeout_ms;
    taskEXIT_CRITICAL();
}

bool remote_get_state(RemoteState *out)
{
    if (!out) return false;
    uint32_t timeout_ms;
    taskENTER_CRITICAL();
    *out = published;
    timeout_ms = published_timeout_ms;
    taskEXIT_CRITICAL();
    /* 必须在复制之后取时间，防止快照中的接收时间晚于 now。 */
    if (out->age_ms != UINT32_MAX)
        out->age_ms = bsp_time_ms() - out->received_at_ms;
    out->online = out->online && out->age_ms < timeout_ms;
    return out->online;
}
