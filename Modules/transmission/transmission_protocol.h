#ifndef TRANSMISSION_PROTOCOL_H
#define TRANSMISSION_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 这是旧车已有的 BCP 协议，不是裁判系统协议，也没有 CRC。 */
#define TRANSMISSION_HEAD 0xffu
#define TRANSMISSION_BROADCAST 0x00u
#define TRANSMISSION_PC_ADDRESS 0x01u
#define TRANSMISSION_ROBOT_ADDRESS 0x04u
#define TRANSMISSION_GIMBAL_ID 0x20u
#define TRANSMISSION_HEARTBEAT_ID 0xf0u
#define TRANSMISSION_RPY_DATA_SIZE 25u
#define TRANSMISSION_MAX_DATA_SIZE 36u
#define TRANSMISSION_RPY_FRAME_SIZE (TRANSMISSION_RPY_DATA_SIZE + 6u)
#define TRANSMISSION_HEARTBEAT_FRAME_SIZE 7u
#define TRANSMISSION_MAX_FRAME_SIZE (TRANSMISSION_MAX_DATA_SIZE + 6u)

typedef enum
{
    TRANSMISSION_TEAM_UNKNOWN = -1,
    TRANSMISSION_TEAM_BLUE = 0,
    TRANSMISSION_TEAM_RED = 1
} TransmissionTeam;

typedef struct
{
    uint8_t destination, id, length;
    uint8_t data[TRANSMISSION_MAX_DATA_SIZE];
} TransmissionFrame;

typedef struct
{
    uint8_t bytes[TRANSMISSION_MAX_FRAME_SIZE];
    size_t used;
    uint32_t frames, checksum_errors, length_errors, discarded_bytes;
} TransmissionParser;

typedef struct
{
    float yaw_deg, pitch_deg;
    bool fire_advice;
    uint8_t raw_target_flag; /* 旧程序没有解释这个字节；原样交给调试界面。 */
} TransmissionTarget;

typedef void (*TransmissionFrameHandler)(const TransmissionFrame *frame, void *context);
void transmission_parser_reset(TransmissionParser *parser);
void transmission_parser_feed(TransmissionParser *parser, const uint8_t *data, size_t length,
                              TransmissionFrameHandler handler, void *context);
bool transmission_decode_target(const TransmissionFrame *frame, TransmissionTarget *out);
/* yaw/pitch 单位是度。浮点数先乘 1000 变成整数，再按小端写入，不直接传 float。 */
size_t transmission_encode_attitude(uint8_t *out, size_t capacity, float yaw_deg,
                                    float pitch_deg, TransmissionTeam team);
/* OMNI9_26扩展：保留31字节长度和原角度/队伍位置，DATA[17..24]改传角速度。
 * 角速度单位度/秒，乘1000写有符号小端整数；旧RTT没有定义这两个保留字段。
 * 角度/速度使用同一套NUC方向，调用者负责方向和rad/s到deg/s换算。 */
size_t transmission_encode_motion(uint8_t *out, size_t capacity, float yaw_deg,
                                  float pitch_deg, float yaw_rate_deg_s,
                                  float pitch_rate_deg_s, TransmissionTeam team);
/* MCU->NUC短心跳：FF 01 F0 01 sequence SC AC；序号按uint8循环。 */
size_t transmission_encode_heartbeat(uint8_t *out, size_t capacity, uint8_t sequence);
bool transmission_checksum_valid(const uint8_t *frame, size_t length);

#endif
