#ifndef MODULES_REMOTE_REMOTE_H
#define MODULES_REMOTE_REMOTE_H

#include "dbus_protocol.h"
typedef struct {
    DbusState channels;
    uint32_t received_at_ms, age_ms;
    bool online;
} RemoteState;
typedef struct {
    int16_t ch1, ch2, ch3, ch4;
    uint8_t sw1, sw2;
    uint32_t online, enabled, age_ms, valid_frames, invalid_frames;
} RemoteDebug;
extern volatile RemoteDebug debug_remote;
bool remote_init(void);
/* 唯一调用者 cmd_task；接收/解码/失联判定一次，复制结果，不创建线程。 */
void remote_update(uint32_t timeout_ms, RemoteState *out);
/* 其他任务只读已发布的遥控状态，不取走接收队列中的数据。
 * 返回值=目前在线；每次读取都会重新计算帧龄，cmd_task 停更后也会超时。
 * 使用 remote_update 的超时设置。离线时 out 中可能保留旧通道，不能用于驱动。
 * out 必须有效；只能在任务上下文调用。 */
bool remote_get_state(RemoteState *out);

#endif
