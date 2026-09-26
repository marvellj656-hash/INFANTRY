#ifndef INS_TASK_H
#define INS_TASK_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
  float yaw_deg, pitch_deg, yaw_rate_rad_s, pitch_rate_rad_s;
  uint32_t received_at_ms, generation;
  bool valid;
} InsState;
/* 复制姿态，读取时重新检查 20ms 新鲜度；不让其他任务访问 EKF 的中间结果。 */
typedef struct {
  uint32_t initialized, valid, stationary_samples, sample_errors, generation;
  float accel_norm, gyro_norm, yaw_deg, pitch_deg;
} InsDebug;
extern volatile InsDebug debug_ins;
bool ins_get_state(InsState *out);
void ins_task_entry(void *argument);
#endif
