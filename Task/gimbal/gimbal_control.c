#include "gimbal_control.h"
#include "gimbal_config.h"
#include <math.h>
#include <string.h>
static float clamp(float x, float lo, float hi) {
  return fminf(hi, fmaxf(lo, x));
}
static void configure(GimbalController *s, bool automatic) {
  const unsigned opt = PID_Trapezoid_Intergral | PID_Integral_Limit |
                       PID_Derivative_On_Measurement;
  /* 每个模式都完整读取自己的五个参数，避免“头文件改了但这里仍写死0”。 */
  const pid_config_t a[2] = {
      INIT_PID_CONFIG(
          automatic ? YAW_KP_A_AUTO : YAW_KP_A_IMU,
          automatic ? YAW_KI_A_AUTO : YAW_KI_A_IMU,
          automatic ? YAW_KD_A_AUTO : YAW_KD_A_IMU,
          automatic ? YAW_INTEGRAL_A_AUTO : YAW_INTEGRAL_A_IMU,
          automatic ? YAW_MAX_A_AUTO : YAW_MAX_A_IMU,
          opt),
      INIT_PID_CONFIG(
          automatic ? PITCH_KP_A_AUTO : PITCH_KP_A_IMU,
          automatic ? PITCH_KI_A_AUTO : PITCH_KI_A_IMU,
          automatic ? PITCH_KD_A_AUTO : PITCH_KD_A_IMU,
          automatic ? PITCH_INTEGRAL_A_AUTO : PITCH_INTEGRAL_A_IMU,
          automatic ? PITCH_MAX_A_AUTO : PITCH_MAX_A_IMU,
          opt)
  };
  const pid_config_t v[2] = {
      INIT_PID_CONFIG(
          automatic ? YAW_KP_V_AUTO : YAW_KP_V_IMU,
          automatic ? YAW_KI_V_AUTO : YAW_KI_V_IMU,
          automatic ? YAW_KD_V_AUTO : YAW_KD_V_IMU,
          automatic ? YAW_INTEGRAL_V_AUTO : YAW_INTEGRAL_V_IMU,
          automatic ? YAW_MAX_V_AUTO : YAW_MAX_V_IMU,
          opt),
      INIT_PID_CONFIG(
          automatic ? PITCH_KP_V_AUTO : PITCH_KP_V_IMU,
          automatic ? PITCH_KI_V_AUTO : PITCH_KI_V_IMU,
          automatic ? PITCH_KD_V_AUTO : PITCH_KD_V_IMU,
          automatic ? PITCH_INTEGRAL_V_AUTO : PITCH_INTEGRAL_V_IMU,
          automatic ? PITCH_MAX_V_AUTO : PITCH_MAX_V_IMU,
          opt)
  };
  for (unsigned i = 0; i < 2; i++) {
    pid_init(&s->angle[i], &a[i]);
    pid_init(&s->speed[i], &v[i]);
  }
}
void gimbal_control_reset(GimbalController *s) {
  memset(s, 0, sizeof(*s));
  configure(s, false);
}
float gimbal_encoder_relative(uint16_t ecd, uint16_t center, float sign) {
  int d = (int)ecd - (int)center;
  if (d > 4096)
    d -= 8192;
  if (d < -4096)
    d += 8192;
  return sign * (float)d * (360.0f / 8192.0f);
}
void gimbal_control_update(GimbalController *s, const GimbalCommand *c,
                           const GimbalInput *in, bool permit, float dt,
                           GimbalOutput *out) {
  memset(out, 0, sizeof(*out));
  bool finite = isfinite(dt) && dt > 0 && dt <= 0.02f &&
                isfinite(c->yaw_rate_deg_s) && isfinite(c->pitch_rate_deg_s) &&
                isfinite(c->auto_yaw_deg) && isfinite(c->auto_pitch_deg);
  for (unsigned i = 0; i < 2; i++)
    finite = finite && isfinite(in->relative_deg[i]) &&
             isfinite(in->angle_deg[i]) && isfinite(in->rate_rad_s[i]);
  /* 总开关关掉才解除归中超时锁定；传感器掉线不能偷偷重试撞限位。 */
  if (!c->enabled) {
    gimbal_control_reset(s);
    return;
  }
  if (s->fault_latched) {
    out->mode = GIMBAL_FAULT;
    return;
  }
  if (!permit || !in->valid || !finite) {
    gimbal_control_reset(s);
    return;
  }
  if (s->mode == GIMBAL_RELAX) {
    if (!c->neutral)
      return;
    s->mode = GIMBAL_INIT;
    for (unsigned i = 0; i < 2; i++)
      s->start_deg[i] = in->relative_deg[i];
    configure(s, false);
  }
  if (s->mode == GIMBAL_INIT) {
    s->homing_s += dt;
    float fraction =
        clamp(1.0f - s->homing_s / (BACK_CENTER_TIME * 0.001f), 0, 1);
    for (unsigned i = 0; i < 2; i++)
      s->target_deg[i] = s->start_deg[i] * fraction;
    bool centered =
        fabsf(in->relative_deg[0]) < 3.6f && fabsf(in->relative_deg[1]) < 0.9f;
    if (centered && fraction == 0) {
      s->mode = GIMBAL_GYRO;
      /* 切到惯性角度前接住当前姿态，不能把编码器零度当作 INS 零度。 */
      for (unsigned i = 0; i < 2; i++)
        s->target_deg[i] = s->reference_deg[i] = in->angle_deg[i];
      configure(s, false);
    } else if (s->homing_s >= GIMBAL_HOME_TIMEOUT_S) {
      s->fault_latched = true;
      s->mode = GIMBAL_FAULT;
      out->mode = s->mode;
      return;
    }
  } else {
    GimbalMode next = c->auto_aim ? GIMBAL_AUTO : GIMBAL_GYRO;
    if (next != s->mode) {
      configure(s, c->auto_aim);
      for (unsigned i = 0; i < 2; i++)
        s->target_deg[i] = in->angle_deg[i];
      s->mode = next;
    }
    if (c->auto_aim) {
      s->target_deg[0] = c->auto_yaw_deg;
      s->target_deg[1] = c->auto_pitch_deg;
    } else {
      s->target_deg[0] += clamp(c->yaw_rate_deg_s, -GIMBAL_MAX_RATE_DEG_S,
                                GIMBAL_MAX_RATE_DEG_S) *
                          dt;
      s->target_deg[1] += clamp(c->pitch_rate_deg_s, -GIMBAL_MAX_RATE_DEG_S,
                                GIMBAL_MAX_RATE_DEG_S) *
                          dt;
    }
    s->target_deg[1] = clamp(s->target_deg[1], PIT_ANGLE_MIN, PIT_ANGLE_MAX);
  }
  for (unsigned i = 0; i < 2; i++) {
    float measured =
        s->mode == GIMBAL_INIT ? in->relative_deg[i] : in->angle_deg[i];
    /* 角度环：度 -> rad/s；速度环：rad/s -> GM6020 电压码。沿用旧版参数单位。
     */
    float rate = pid_calculate(&s->angle[i], measured, s->target_deg[i], dt);
    float raw = pid_calculate(&s->speed[i], in->rate_rad_s[i], rate, dt);
    if (i == 0)
      raw = -raw; /* 旧接线：Yaw 电机正方向与云台角度正方向相反。 */
    /* 硬限位按编码器相对角判断，避免姿态零点变化突破机械范围。 */
    if (i == 1 && ((in->relative_deg[1] >= PIT_ANGLE_MAX && raw > 0) ||
                   (in->relative_deg[1] <= PIT_ANGLE_MIN && raw < 0))) {
      raw = 0;
      pid_clear(&s->speed[i]);
      pid_clear(&s->angle[i]);
    }
    out->raw[i] = (int16_t)raw;
    out->target_deg[i] = s->target_deg[i];
    out->relative_deg[i] = in->relative_deg[i];
  }
  out->reference_yaw_deg = s->reference_deg[0];
  out->reference_pitch_deg = s->reference_deg[1];
  out->reference_valid = s->mode == GIMBAL_GYRO || s->mode == GIMBAL_AUTO;
  out->mode = s->mode;
  out->centered = s->mode == GIMBAL_GYRO || s->mode == GIMBAL_AUTO;
}
