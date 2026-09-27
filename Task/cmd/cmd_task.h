#ifndef APP_CMD_CMD_TASK_H
#define APP_CMD_CMD_TASK_H

#include "omni_kinematics.h"
#include <stdint.h>
ChassisVelocity cmd_build_chassis_command(int16_t ch1, int16_t ch2,
                                          int16_t ch3);
typedef struct {
  uint32_t online, enabled, sw1, sw2, spin, friction, fire, neutral, auto_aim;
  int32_t ch1, ch2, ch3, ch4, wheel;
  uint32_t follow;
  uint32_t auto_requested, auto_block_flags;
  float auto_yaw_deg, auto_pitch_deg; /* 仅auto_aim=1时是实际提交的自瞄目标。 */
  float yaw_rate_deg_s, pitch_rate_deg_s; /* 解释了摇杆和鼠标后的手动角速度要求。 */
} CmdDebug;
extern volatile CmdDebug debug_cmd;
void cmd_task_entry(void *argument);

#endif
