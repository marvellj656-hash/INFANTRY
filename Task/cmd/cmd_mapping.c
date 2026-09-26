#include "cmd_mapping.h"
#include "chassis/chassis_config.h"
#include "cmd_task.h"
#include <math.h>
#include <string.h>
static float clamp(float x, float limit) {
  if (x > limit)
    return limit;
  if (x < -limit)
    return -limit;
  return x;
}

static float channel_axis(int16_t channel) {
  float x = clamp((float)channel, 660);
  if (x > 5)
    return (x - 5) / 655.0f;
  if (x < -5)
    return (x + 5) / 655.0f;
  return 0;
}

ChassisVelocity cmd_build_chassis_command(int16_t ch1, int16_t ch2,
                                          int16_t ch3) {
  ChassisVelocity v = {channel_axis(ch1), channel_axis(ch2), channel_axis(ch3)};
  float length = hypotf(v.vx, v.vy);
  if (length > 1) {
    v.vx /= length;
    v.vy /= length;
  }
  /* 在通道映射处使用低速上限，而不是末端硬截断，保留整个摇杆行程。 */
  v.vx *= CHASSIS_MAX_SPEED_MM_S;
  v.vy *= CHASSIS_MAX_SPEED_MM_S;
  v.wz *= CHASSIS_MAX_YAW_RAD_S;
  return v;
}

void cmd_map_update(CmdMapper *s, const CmdInput *in, CmdOutput *out) {
  memset(out, 0, sizeof(*out));
  const RemoteState *r = &in->remote;
  const DbusState *c = &r->channels;
  /* 本工程 sw1=左拨杆（协议 S2），sw2=右拨杆（协议 S1）。
   * 只有合法开关值才能使能；失联时旧通道值仍可显示，但不能驱动车。 */
  bool left_valid = c->sw1 >= RC_SWITCH_UP && c->sw1 <= RC_SWITCH_MIDDLE;
  bool enabled = r->online && left_valid &&
                 (c->sw2 == RC_SWITCH_MIDDLE || c->sw2 == RC_SWITCH_DOWN);
  bool neutral = channel_axis(c->ch[0]) == 0 && channel_axis(c->ch[1]) == 0 &&
                 channel_axis(c->ch[2]) == 0 && channel_axis(c->ch[3]) == 0 &&
                 c->mouse_x == 0 && c->mouse_y == 0;
  uint16_t pressed = c->keys & (uint16_t)~s->previous_keys;
  out->redraw_ui = r->online && (pressed & (1u << 8)) != 0;
  if (!enabled || c->sw1 != s->previous_left_switch) {
    s->friction_inhibit = false;
    s->continuous_key = false;
  }
  if (enabled) {
    if (pressed & (1u << 9))
      s->friction_inhibit = false; /* F：恢复 */
    if (c->keys & (1u << 10))
      s->friction_inhibit = true; /* G：关闭，优先级最高 */
    if (pressed & (1u << 14))
      s->continuous_key = !s->continuous_key; /* V */
  }
  s->previous_keys = c->keys;
  s->previous_left_switch = c->sw1;
  out->spin = enabled && c->sw2 == RC_SWITCH_DOWN;
  out->chassis = (ChassisCommand){
      /* CH3 只转云台。小陀螺由右拨杆选择，避免一根摇杆同时转两套机构。 */
      .target = cmd_build_chassis_command(c->ch[0], c->ch[1], 0),
      .received_at_ms = r->received_at_ms,
      .enabled = enabled,
      .neutral = neutral};
  if (out->spin)
    out->chassis.target.wz = 4.5f;
  out->gimbal = (GimbalCommand){
      .enabled = enabled,
      .neutral = neutral,
      .yaw_rate_deg_s = channel_axis(c->ch[2]) * 180.0f + c->mouse_x * 0.3f,
      .pitch_rate_deg_s = channel_axis(c->ch[3]) * 180.0f + c->mouse_y * 0.2f,
      .received_at_ms = r->received_at_ms};
  bool auto_aim = enabled && c->mouse_right && in->gimbal_fresh &&
                  in->gimbal.centered && in->gimbal.reference_valid &&
                  in->vision.aim_valid;
  if (auto_aim) {
    /* 沿用旧 NUC 接口：角度是相对于进入 AUTO 时的手动目标。
     * 只在入口记一次，不能每圈累加同一包偏移。重连/IMU重置后重建参考。 */
    if (!s->auto_active ||
        s->reference_generation != in->gimbal.reference_generation ||
        s->vision_generation != in->vision.link_generation) {
      s->auto_origin_yaw_deg = in->gimbal.target_deg[0];
      s->auto_origin_pitch_deg = in->gimbal.target_deg[1];
      s->reference_generation = in->gimbal.reference_generation;
      s->vision_generation = in->vision.link_generation;
    }
    out->gimbal.auto_aim = true;
    out->gimbal.auto_yaw_deg = s->auto_origin_yaw_deg + in->vision.yaw_deg;
    out->gimbal.auto_pitch_deg =
        s->auto_origin_pitch_deg + in->vision.pitch_deg;
  }
  s->auto_active = auto_aim;
  bool friction = enabled && c->sw1 != RC_SWITCH_UP && !s->friction_inhibit;
  out->shoot = (ShootCommand){
      .enabled = enabled,
      .friction = friction,
      .fire = enabled && (c->mouse_left || c->wheel >= 200 || c->wheel <= -200),
      .mode = c->sw1 == RC_SWITCH_DOWN || s->continuous_key ? SHOOT_CONTINUOUS
                                                            : SHOOT_SINGLE,
      .received_at_ms = r->received_at_ms};
  if (c->wheel <= -200)
    out->shoot.mode = SHOOT_REVERSE;
  /* NUC 的 fire_advice 只作为观测值，不能直接当成操作者按下扳机。 */
  if (!enabled) {
    out->chassis.target = (ChassisVelocity){0};
    out->gimbal.yaw_rate_deg_s = out->gimbal.pitch_rate_deg_s = 0;
    out->shoot.mode = SHOOT_STOP;
  }
}
