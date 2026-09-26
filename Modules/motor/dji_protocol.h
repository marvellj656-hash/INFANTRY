#ifndef MODULES_MOTOR_DJI_PROTOCOL_H
#define MODULES_MOTOR_DJI_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm, current_raw;
    uint8_t temperature;
} DjiFeedback;
bool dji_motor_decode_feedback(const uint8_t *data, size_t length, DjiFeedback *out);
/* limits 是该组四个槽位的允许码值，未配置的槽位必须传 0。 */
void dji_motor_encode_currents(const int16_t raw[4], const int16_t limits[4], uint8_t bytes[8]);

#endif
