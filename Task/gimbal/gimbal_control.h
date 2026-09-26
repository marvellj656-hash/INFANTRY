#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H
#include "pid.h"
#include <stdbool.h>
#include <stdint.h>
typedef enum {
  GIMBAL_RELAX,
  GIMBAL_INIT,
  GIMBAL_GYRO,
  GIMBAL_AUTO,
  GIMBAL_FAULT
} GimbalMode;
typedef struct {
  bool enabled, neutral, auto_aim;
  float yaw_rate_deg_s, pitch_rate_deg_s;
  float auto_yaw_deg, auto_pitch_deg; /* 与 INS 相同参考系的绝对角度。 */
  uint32_t received_at_ms;
} GimbalCommand;
typedef struct {
  float relative_deg[2], angle_deg[2], rate_rad_s[2];
  bool valid;
} GimbalInput;
typedef struct {
  pid_obj_t angle[2], speed[2];
  GimbalMode mode;
  float target_deg[2], start_deg[2], homing_s;
  float reference_deg[2];
  bool fault_latched;
} GimbalController;
typedef struct {
  int16_t raw[2];
  GimbalMode mode;
  bool centered;
  float target_deg[2], relative_deg[2];
  float reference_yaw_deg, reference_pitch_deg;
  uint32_t reference_generation;
  bool reference_valid;
} GimbalOutput;
void gimbal_control_reset(GimbalController *s);
void gimbal_control_update(GimbalController *s, const GimbalCommand *cmd,
                           const GimbalInput *in, bool permit, float dt,
                           GimbalOutput *out);
/* 编码器绕回处也走最短角度，例如 8190 和 2 只差 4 格。 */
float gimbal_encoder_relative(uint16_t ecd, uint16_t center, float sign);
#endif
