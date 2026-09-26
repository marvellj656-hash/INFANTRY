#include "dbus_protocol.h"
static uint16_t read16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static int16_t signed16(const uint8_t *p)
{
    uint16_t value = read16(p);
    return (int16_t)((value & 0x8000u) ? (int32_t)value - 65536 : value);
}
bool dbus_decode(const uint8_t *b, size_t length, DbusState *out)
{
    if (!b || !out || length != DBUS_FRAME_BYTES) return false;
    DbusState s = {0};
    s.ch[0] = (int16_t)((int32_t)(read16(b) & 0x7ffu) - 1024);
    s.ch[1] = (int16_t)((int32_t)((read16(b + 1) >> 3) & 0x7ffu) - 1024);
    s.ch[2] = (int16_t)((int32_t)(((uint32_t)b[2] >> 6 |
                          (uint32_t)b[3] << 2 |
                          (uint32_t)b[4] << 10) & 0x7ffu) - 1024);
    s.ch[3] = (int16_t)((int32_t)((read16(b + 4) >> 1) & 0x7ffu) - 1024);
    s.sw1 = (b[5] >> 6) & 3u;
    s.sw2 = (b[5] >> 4) & 3u;
    for (unsigned i = 0; i < 4; ++i)
        if (s.ch[i] < -660 || s.ch[i] > 660) return false;
    if (s.sw1 < 1 || s.sw1 > 3 || s.sw2 < 1 || s.sw2 > 3 ||
        b[12] > 1 || b[13] > 1) return false;
    s.mouse_x = signed16(b + 6);
    s.mouse_y = signed16(b + 8);
    s.mouse_z = signed16(b + 10);
    s.mouse_left = b[12]; s.mouse_right = b[13];
    s.keys = read16(b + 14);
    s.wheel = (int16_t)((int32_t)read16(b + 16) - 1024);
    *out = s;
    return true;
}
