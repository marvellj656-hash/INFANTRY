#ifndef DJI_FEEDBACK_H
#define DJI_FEEDBACK_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#define DJI_FEEDBACK_MOTORS 6u
typedef struct {
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t current_raw;
    uint8_t temperature;
} DjiFeedback;
/* 0..3: chassis, 4: yaw, 5: pitch. bus is 1 or 2. */
int dji_feedback_index(uint8_t bus, uint16_t id);
bool dji_feedback_decode(const uint8_t *data, size_t length, DjiFeedback *out);
#endif
