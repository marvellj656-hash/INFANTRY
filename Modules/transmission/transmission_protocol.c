#include "transmission_protocol.h"
#include <math.h>
#include <string.h>

static void checksum(const uint8_t *frame, size_t data_size, uint8_t *sum, uint8_t *add)
{
    /* 老协议特殊之处：4 字节头先一起加，AC 只收一次这个和。
     * 不能换成 CRC，也不能对头的每个字节依次做二重累加。 */
    *sum = (uint8_t)(frame[0] + frame[1] + frame[2] + frame[3]);
    *add = *sum;
    for (size_t i = 0; i < data_size; ++i)
    {
        *sum = (uint8_t)(*sum + frame[4u + i]);
        *add = (uint8_t)(*add + *sum);
    }
}

bool transmission_checksum_valid(const uint8_t *frame, size_t length)
{
    if (!frame || length < 6u || frame[0] != TRANSMISSION_HEAD ||
        frame[3] > TRANSMISSION_MAX_DATA_SIZE || length != (size_t)frame[3] + 6u)
        return false;
    uint8_t sum, add;
    checksum(frame, frame[3], &sum, &add);
    return frame[length - 2u] == sum && frame[length - 1u] == add;
}

void transmission_parser_reset(TransmissionParser *p)
{
    if (p)
        memset(p, 0, sizeof(*p));
}

static void discard(TransmissionParser *p, size_t count)
{
    memmove(p->bytes, p->bytes + count, p->used - count);
    p->used -= count;
}

void transmission_parser_feed(TransmissionParser *p, const uint8_t *data, size_t length,
                              TransmissionFrameHandler handler, void *context)
{
    if (!p || (!data && length))
        return;
    for (size_t i = 0; i < length; ++i)
    {
        /* 逐字节推进，所以一次 USB 包含半帧、两帧都可以，不靠 USB 包边界。 */
        if (p->used == sizeof(p->bytes))
        {
            discard(p, 1u);
            p->discarded_bytes++;
        }
        p->bytes[p->used++] = data[i];
        while (p->used)
        {
            if (p->bytes[0] != TRANSMISSION_HEAD)
            {
                discard(p, 1u);
                p->discarded_bytes++;
                continue;
            }
            if (p->used < 4u)
                break;
            if (p->bytes[3] > TRANSMISSION_MAX_DATA_SIZE)
            {
                discard(p, 1u);
                p->length_errors++;
                continue;
            }
            size_t frame_size = (size_t)p->bytes[3] + 6u;
            if (p->used < frame_size)
                break;
            if (!transmission_checksum_valid(p->bytes, frame_size))
            {
                /* 校验失败只挪走一个字节。旧程序一下丢 31 字节，会把后面的好帧丢掉。 */
                discard(p, 1u);
                p->checksum_errors++;
                continue;
            }
            TransmissionFrame frame = {.destination = p->bytes[1], .id = p->bytes[2],
                                       .length = p->bytes[3]};
            memcpy(frame.data, p->bytes + 4u, frame.length);
            p->frames++;
            if (handler)
                handler(&frame, context);
            discard(p, frame_size);
        }
    }
}

static int32_t read_i32_le(const uint8_t *data)
{
    uint32_t raw = (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
                   ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
    int32_t value;
    /* memcpy 不要求数据地址对齐；不会产生旧代码 int32_t* 强转的别名问题。 */
    memcpy(&value, &raw, sizeof(value));
    return value;
}

bool transmission_decode_target(const TransmissionFrame *frame, TransmissionTarget *out)
{
    if (!frame || !out || frame->id != TRANSMISSION_GIMBAL_ID ||
        frame->length != TRANSMISSION_RPY_DATA_SIZE)
        return false;
    TransmissionTarget target = {
        .yaw_deg = -(float)read_i32_le(frame->data + 1u) / 1000.0f,
        .pitch_deg = (float)read_i32_le(frame->data + 5u) / 1000.0f,
        .raw_target_flag = frame->data[0]};
    int32_t fire = read_i32_le(frame->data + 9u);
    /* 大幅错误先挡在模块门口；机械限位仍由云台自己的控制器负责。
     * fire 沿用旧约定 0/1000，不能把任意非零损坏数据当作允许开火。 */
    if (fabsf(target.yaw_deg) > 3600.0f || fabsf(target.pitch_deg) > 180.0f ||
        (fire != 0 && fire != 1000))
        return false;
    target.fire_advice = fire == 1000;
    *out = target;
    return true;
}

static void write_i32_le(uint8_t *out, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    for (unsigned i = 0; i < 4u; ++i)
        out[i] = (uint8_t)(raw >> (8u * i));
}

size_t transmission_encode_attitude(uint8_t *out, size_t capacity, float yaw_deg,
                                    float pitch_deg, TransmissionTeam team)
{
    if (!out || capacity < TRANSMISSION_RPY_FRAME_SIZE || !isfinite(yaw_deg) ||
        !isfinite(pitch_deg) || fabsf(yaw_deg) > 2000000.0f ||
        fabsf(pitch_deg) > 180.0f || team < TRANSMISSION_TEAM_UNKNOWN ||
        team > TRANSMISSION_TEAM_RED)
        return 0u;
    memset(out, 0, TRANSMISSION_RPY_FRAME_SIZE);
    out[0] = TRANSMISSION_HEAD;
    out[1] = TRANSMISSION_PC_ADDRESS;
    out[2] = TRANSMISSION_GIMBAL_ID;
    out[3] = TRANSMISSION_RPY_DATA_SIZE;
    write_i32_le(out + 5u, (int32_t)(yaw_deg * 1000.0f));
    write_i32_le(out + 9u, (int32_t)(pitch_deg * 1000.0f));
    /* DATA[9..12] 固定 0：旧代码本来发 0，不把本机扳机状态擅自塞进协议。 */
    write_i32_le(out + 17u, (int32_t)team * 1000);
    checksum(out, TRANSMISSION_RPY_DATA_SIZE, out + TRANSMISSION_RPY_FRAME_SIZE - 2u,
             out + TRANSMISSION_RPY_FRAME_SIZE - 1u);
    return TRANSMISSION_RPY_FRAME_SIZE;
}
