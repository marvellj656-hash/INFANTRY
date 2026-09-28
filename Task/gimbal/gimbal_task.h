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
  GIMBAL_HOME_FAILED = 64u,
  GIMBAL_BLOCK_OUTPUT = 128u
};
typedef struct {
  uint32_t faults, mode, imu_valid, neutral, yaw_online, pitch_online, tx_ready,
      command_age_ms;
  int32_t yaw_output, pitch_output;
  float yaw_relative_deg, pitch_relative_deg;
  /* 最终目标：GYRO/AUTO与INS同参考；INIT归中阶段使用编码器参考。 */
  float yaw_target_deg, pitch_target_deg;
  float pitch_measured_deg, pitch_error_deg;
  float pitch_rate_target_rad_s, pitch_rate_measured_rad_s;
  float pitch_speed_p, pitch_speed_i, pitch_speed_d;
  uint32_t pitch_ecd, pitch_target_limited, pitch_output_limited;
  uint32_t last_faults, fault_cycles; /* 保留最近一次运行故障，避免瞬间掉线看不到。 */
} GimbalDebug;
extern volatile GimbalDebug debug_gimbal;
bool gimbal_init(void);
bool gimbal_set_command(const GimbalCommand *command);
bool gimbal_get_state(
    GimbalOutput *out); /* 返回状态是否新鲜；不是指向内部的指针。 */
void gimbal_task_entry(void *argument);
#endif
