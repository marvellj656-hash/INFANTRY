#ifndef DBUS_PROTOCOL_H
#define DBUS_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define DBUS_FRAME_BYTES 18u
typedef struct {
    int16_t ch[4];
    uint8_t sw1, sw2;
    int16_t mouse_x, mouse_y, mouse_z;
    uint8_t mouse_left, mouse_right;
    uint16_t keys;
    int16_t wheel;
} DbusState;
/* No HAL/RTOS dependencies. Invalid input leaves *out unchanged. */
bool dbus_decode(const uint8_t *bytes, size_t length, DbusState *out);
#endif
