#ifndef TRANSMISSION_TX_H
#define TRANSMISSION_TX_H

#include "transmission_protocol.h"

#define TRANSMISSION_ATTITUDE_PERIOD_MS 10u
#define TRANSMISSION_HEARTBEAT_PERIOD_MS 100u
/* 新NUC需按LEN分帧并识别F0；固定只读31字节的旧程序可暂设0做兼容检查。 */
#ifndef TRANSMISSION_TX_HEARTBEAT_ENABLE
#define TRANSMISSION_TX_HEARTBEAT_ENABLE 1
#endif

/* 已换到 NUC 使用的方向；四个量分别是度和度/秒，不是电机编码器值。 */
typedef struct
{
    float yaw_deg, pitch_deg, yaw_rate_deg_s, pitch_rate_deg_s;
    TransmissionTeam team;
} TransmissionMotion;

/* true 表示运输层已经复制并接受这帧，不表示 NUC 已经处理完。 */
typedef bool (*TransmissionSend)(const uint8_t *data, size_t length, void *context);

typedef struct
{
    uint32_t last_motion_ms, last_heartbeat_ms;
    uint32_t frames, busy, invalid_samples, heartbeat_frames, heartbeat_busy;
    TransmissionMotion last_motion;
    uint8_t heartbeat, next_heartbeat;
    bool motion_sent, heartbeat_sent, ready, reference_valid;
} TransmissionTx;

void transmission_tx_init(TransmissionTx *tx);
/* 每次最多交一帧：到期心跳优先，否则发送姿态。NULL姿态仍然可以发心跳。
 * USB忙时立即返回，下次用最新姿态重试，不排长队、不发送积压旧角度。 */
void transmission_tx_update(TransmissionTx *tx, uint32_t now_ms, bool ready,
                            const TransmissionMotion *motion,
                            TransmissionSend send, void *context);

#endif
