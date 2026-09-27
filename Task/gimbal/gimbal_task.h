#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include "gimbal_control.h"
enum {
  GIMBAL_BLOCK_COMMAND = 1u,
  GIMBAL_BLOCK_MOTOR = 2u,
  GIMBAL_BLOCK_CAN = 4u,
  GIMBAL_BLOCK_IMU = 8u,
  GIMBAL_BLOCK_TIMING = 16u,
  GIMBAL_WAIT_NEUTRAL = 32u,
  GIMBAL_HOME_FAILED = 64u
};
typedef struct {
  uint32_t faults, mode, imu_valid, neutral, yaw_online, pitch_online, tx_ready,
      command_age_ms;
  int32_t yaw_output, pitch_output;
  float yaw_relative_deg, pitch_relative_deg;
  /* 最终目标：GYRO/AUTO与INS同参考；INIT归中阶段使用编码器参考。 */
  float yaw_target_deg, pitch_target_deg;
} GimbalDebug;
extern volatile GimbalDebug debug_gimbal;
bool gimbal_init(void);
bool gimbal_set_command(const GimbalCommand *command);
bool gimbal_get_state(
    GimbalOutput *out); /* 返回状态是否新鲜；不是指向内部的指针。 */
void gimbal_task_entry(void *argument);
#endif
