#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#include "transmission_protocol.h"

#define TRANSMISSION_TARGET_TIMEOUT_MS 100u
#define TRANSMISSION_LINK_TIMEOUT_MS 500u
#define TRANSMISSION_HEARTBEAT_TIMEOUT_MS 500u

typedef struct
{
    float yaw_deg, pitch_deg; /* 已按旧程序转换符号；相对于与 NUC 约定的参考角。 */
    uint32_t received_at_ms, last_frame_at_ms, age_ms, command_generation, link_generation;
    uint32_t heartbeat_at_ms, heartbeat_age_ms, heartbeat_frames;
    uint8_t raw_target_flag, heartbeat, destination;
    bool online, aim_valid, fire_advice, heartbeat_online;
} TransmissionState;

typedef struct
{
    uint32_t rx_bytes, valid_frames, target_frames, heartbeat_frames, invalid_frames;
    uint32_t checksum_errors, length_errors, discarded_bytes, generation_resets;
    uint32_t target_timeouts, tx_frames, tx_busy, age_ms, command_generation;
    uint32_t heartbeat_at_ms, heartbeat_age_ms;
    uint32_t tx_heartbeat_frames, tx_heartbeat_busy, tx_invalid_samples;
    uint32_t tx_attitude_age_ms;
    float yaw_deg, pitch_deg;
    /* 只在USB接受帧后更新：这些值代表已交给USB的样本，不是假定NUC收到。 */
    float tx_yaw_deg, tx_pitch_deg, tx_yaw_rate_deg_s, tx_pitch_rate_deg_s;
    uint8_t raw_target_flag, heartbeat, destination, online, aim_valid, fire_advice;
    uint8_t heartbeat_online, tx_heartbeat;
    uint8_t tx_ready, tx_reference_valid;
} TransmissionDebug;

extern volatile TransmissionDebug debug_transmission;
/* 读到的是副本。返回 true 仅表示有新鲜可用的角度包；不代表视觉确认识别到了目标。
 * 旧 DATA[0] 的含义未提供；模块保留原值，绝不把它私自当成“有目标”开关。 */
bool transmission_get_state(TransmissionState *out);

#endif
