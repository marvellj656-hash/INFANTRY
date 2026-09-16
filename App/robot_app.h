#ifndef ROBOT_APP_H
#define ROBOT_APP_H
#include <stdbool.h>
#include <stdint.h>
#include "motion_command.h"
#include "chassis_control.h"
typedef struct {
    int16_t ch1, ch2, ch3, ch4;
    uint8_t sw1, sw2;
    uint32_t online, enabled, age_ms, valid_frames, invalid_frames;
} RemoteDebug;
typedef struct {
    PlanarVelocity target, output;
    uint32_t enabled, cycles, timing_faults;
} ControlDebug;
extern volatile RemoteDebug debug_remote;
extern volatile ControlDebug debug_control;
extern volatile ChassisOutput debug_chassis;
bool robot_app_init(void);
#endif
