#include "dji_protocol.h"

/* DJI 电机反馈是大端：高字节在前；不要直接复用 DBUS 的小端 read16。 */
static uint16_t read_be16(const uint8_t *p)
{
    return ((uint16_t)p[0] << 8) | p[1];
}

/* 把补码表示的负转速/电流还原成有符号数。 */
static int16_t read_signed_be16(const uint8_t *p)
{
    uint16_t value = read_be16(p);
    return (int16_t)((value & 0x8000u) ? (int32_t)value - 65536 : value);
}

bool dji_motor_decode_feedback(const uint8_t *data, size_t length, DjiFeedback *out)
{
    if (!data || !out || length != 8)
        return false;
    DjiFeedback value = {0};
    value.ecd = read_be16(data); /* byte 0/1：编码器，0..8191 */
    if (value.ecd > 8191)
        return false;
    value.speed_rpm = read_signed_be16(data + 2);   /* byte 2/3：转子 RPM */
    value.current_raw = read_signed_be16(data + 4); /* byte 4/5：电流原始值，非安培 */
    value.temperature = data[6];                    /* byte 6：温度；byte 7 保留 */
    *out = value; /* 校验通过才更新调用者的结果。这里不操作硬件、不用 RTOS。 */
    return true;
}

void dji_motor_encode_currents(const int16_t raw[4], const int16_t limits[4], uint8_t bytes[8])
{
    /* CAN 标准数据帧 0x200，一帧按顺序控制 C620 ID1..4，都是高字节在前。 */
    for (unsigned i = 0; i < 4; ++i)
    {
        int32_t limited = raw[i];
        int32_t limit = limits[i] > 0 ? limits[i] : 0;
        if (limited > limit)
            limited = limit;
        if (limited < -limit)
            limited = -limit;
        uint16_t word = (uint16_t)(int16_t)limited;
        bytes[2 * i] = (uint8_t)(word >> 8);
        bytes[2 * i + 1] = (uint8_t)word;
    }
}
