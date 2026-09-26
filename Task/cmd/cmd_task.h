#ifndef APP_CMD_CMD_TASK_H
#define APP_CMD_CMD_TASK_H

#include "omni_kinematics.h"
#include <stdint.h>
ChassisVelocity cmd_build_chassis_command(int16_t ch1, int16_t ch2,
                                          int16_t ch3);
typedef struct {
  uint32_t online, enabled, sw1, sw2, spin, friction, fire, neutral, auto_aim;
  int32_t ch1, ch2, ch3, ch4, wheel;
} CmdDebug;
extern volatile CmdDebug debug_cmd;
void cmd_task_entry(void *argument);

#endif
