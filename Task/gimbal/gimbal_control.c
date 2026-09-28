#include "gimbal_control.h"
#include "gimbal_config.h"
#include <math.h>
#include <string.h>
static float clamp(float x, float lo, float hi) {
  return fminf(hi, fmaxf(lo, x));
}
static void configure(GimbalController *s, bool automatic) {
  const unsigned opt = PID_Trapezoid_Intergral | PID_Integral_Limit |
                       PID_Derivative_On_Measurement | PID_RttControl;
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
  if (!c->enabled) {
    gimbal_control_reset(s);
    return;
  }
  if (!permit || !in->valid || !finite) {
    /* 真实失效输出0，但短暂失效不丢掉已归中的模式和目标。 */
    for (unsigned i=0;i<2;i++) { pid_clear(&s->angle[i]); pid_clear(&s->speed[i]); }
    out->mode = s->mode;
    return;
  }
  if (s->mode == GIMBAL_RELAX) {
    s->mode = GIMBAL_INIT;
    configure(s, false);
  }
  bool finish_home = false;
  if (s->mode == GIMBAL_INIT) {
    s->homing_s += dt;
    float fraction = clamp(1.0f-s->homing_s/(BACK_CENTER_TIME*0.001f),0,1);
    /* RTT用本轮编码器角度乘剩余斜坡，不是用刚上电的角度作起点。 */
    for (unsigned i=0;i<2;i++) s->target_deg[i]=in->relative_deg[i]*fraction;
    finish_home = (fabsf(in->relative_deg[0]) <= 80.0f/22.75f &&
                   fabsf(in->relative_deg[1]) <= 20.0f/22.75f) ||
                  (s->homing_s > GIMBAL_HOME_RELAX_AFTER_S &&
                   fabsf(in->relative_deg[0]) <= 200.0f/22.75f &&
                   fabsf(in->relative_deg[1]) <= 200.0f/22.75f);
  } else {
    GimbalMode next = c->auto_aim ? GIMBAL_AUTO : GIMBAL_GYRO;
    if (next != s->mode) {
      /* RTT换PID并清历史，目标承接上次命令，不重新抓取实际角度。 */
      configure(s, c->auto_aim);
      s->mode = next;
    }
    if (c->auto_aim) {
      s->target_deg[0] = c->auto_yaw_deg;
      s->target_deg[1] = c->auto_pitch_deg;
    } else {
      s->target_deg[0] += c->yaw_rate_deg_s*dt;
      s->target_deg[1] += c->pitch_rate_deg_s*dt;
    }
    /* RTT的Pitch限位施加在INS角度目标上，不额外切断电机输出。 */
    float requested=s->target_deg[1];
    s->target_deg[1]=clamp(requested,PIT_ANGLE_MIN,PIT_ANGLE_MAX);
    out->pitch_target_limited=requested!=s->target_deg[1];
  }
  for (unsigned i = 0; i < 2; i++) {
    float measured =
        s->mode == GIMBAL_INIT ? in->relative_deg[i] : in->angle_deg[i];
    /* 角度环：度 -> rad/s；速度环：rad/s -> GM6020 电压码。沿用旧版参数单位。
     */
    float target = s->target_deg[i];
    if (i == 0 && s->mode != GIMBAL_INIT) {
      measured -= s->reference_deg[0];
      target -= s->reference_deg[0];
    }
    float rate = pid_calculate(&s->angle[i], measured, target, dt);
    out->rate_target_rad_s[i] = rate;
    float raw = pid_calculate(&s->speed[i], in->rate_rad_s[i], rate, dt);
    if (i == 0)
      raw = -raw; /* 旧接线：Yaw 电机正方向与云台角度正方向相反。 */
    out->raw[i] = (int16_t)raw;
    out->target_deg[i] = s->target_deg[i];
    out->relative_deg[i] = in->relative_deg[i];
  }
  if (finish_home) {
    /* 本拍仍输出编码器归中控制，下一拍进入INS。Yaw用绝对INS加零点实现等价平移。
     * Pitch沿用RTT：归中后目标为INS 0度，不抓住当前INS姿态。 */
    s->reference_deg[0]=in->angle_deg[0];
    s->reference_deg[1]=in->angle_deg[1];
    s->target_deg[0]=in->angle_deg[0];
    s->target_deg[1]=0;
    s->mode=GIMBAL_GYRO;
    configure(s,false);
    out->target_deg[0]=s->target_deg[0];
    out->target_deg[1]=s->target_deg[1];
  }
  out->reference_yaw_deg = s->reference_deg[0];
  out->reference_pitch_deg = s->reference_deg[1];
  out->reference_valid = s->mode == GIMBAL_GYRO || s->mode == GIMBAL_AUTO;
  out->mode = s->mode;
  out->centered = s->mode == GIMBAL_GYRO || s->mode == GIMBAL_AUTO;
}
