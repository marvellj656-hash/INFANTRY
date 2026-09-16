#include "dji_feedback.h"

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

/* 这里的 ID 沿用原工程。不同 CAN 总线可以存在相同 ID，所以必须一起判断。 */
int dji_feedback_index(uint8_t bus, uint16_t id)
{
    if (bus == 1 && id >= 0x201 && id <= 0x204) return id - 0x201;
    if (bus == 1 && id == 0x207) return 4;
    if (bus == 2 && id == 0x208) return 5;
    return -1;
}

bool dji_feedback_decode(const uint8_t *data, size_t length, DjiFeedback *out)
{
    if (!data || !out || length != 8) return false;
    DjiFeedback value = {0};
    value.ecd = read_be16(data);                     /* byte 0/1：编码器，0..8191 */
    if (value.ecd > 8191) return false;
    value.speed_rpm = read_signed_be16(data + 2);    /* byte 2/3：转子 RPM */
    value.current_raw = read_signed_be16(data + 4);  /* byte 4/5：电流原始值，非安培 */
    value.temperature = data[6];                    /* byte 6：温度；byte 7 保留 */
    *out = value; /* 校验通过才更新调用者的结果。这里不操作硬件、不用 RTOS。 */
    return true;
}
