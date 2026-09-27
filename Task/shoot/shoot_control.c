#include "shoot_control.h"
#include "shoot_config.h"
#include "ramp.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
void shoot_control_reset(ShootController *s)
{
    memset(s, 0, sizeof(*s));
    unsigned opt = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement;
    const pid_config_t friction = INIT_PID_CONFIG(30, 0, 0.08f, 3, 16000, opt);
    const pid_config_t trigger = INIT_PID_CONFIG(10, 5, 0.01f, 1500, 10000, opt);
    const pid_config_t angle = INIT_PID_CONFIG(5, 0, 0, 0, 10000, opt);
    pid_init(&s->speed[0], &friction);
    pid_init(&s->speed[1], &friction);
    pid_init(&s->speed[2], &trigger);
    pid_init(&s->angle, &angle);
    const pid_config_t brake = INIT_PID_CONFIG(
        FRICTION_BRAKE_KP, FRICTION_BRAKE_KI, FRICTION_BRAKE_KD, 0,
        FRICTION_BRAKE_LIMIT_RAW, opt);
    for (unsigned i = 0; i < 2; ++i)
        pid_init(&s->brake[i], &brake);
}
/* 普通“关摩擦轮”先刹车；disable/断线在调用前直接复位，绝不进入这里。
 * 输出只允许与当前转速反向。接近零速就松开，不把停车变成反转。 */
static void brake_update(ShootController *s, const ShootInput *in, float dt, ShootOutput *out)
{
    if (!s->friction_running)
    {
        shoot_control_reset(s);
        return; /* 刚上电时没有转过，不因外力转动而自行通电制动。 */
    }
    s->braking = true;
    s->armed = false;
    s->brake_s += dt;
    bool stopped = fabsf(in->rpm[0]) <= FRICTION_STOP_RPM &&
                   fabsf(in->rpm[1]) <= FRICTION_STOP_RPM;
    s->stopped_s = stopped ? s->stopped_s + dt : 0;
    if (s->stopped_s >= FRICTION_STOP_HOLD_S || s->brake_s >= FRICTION_BRAKE_TIMEOUT_S)
    {
        out->brake_timed_out = s->stopped_s < FRICTION_STOP_HOLD_S;
        shoot_control_reset(s);
        return;
    }
    out->braking = true;
    for (unsigned i = 0; i < 2; ++i)
    {
        if (fabsf(in->rpm[i]) <= FRICTION_STOP_RPM)
        {
            pid_clear(&s->brake[i]);
            continue;
        }
        float raw = pid_calculate(&s->brake[i], in->rpm[i], 0, dt);
        if (raw * in->rpm[i] < 0)
            out->raw[i] = (int16_t)raw;
    }
    /* out 初值全为0：停车期间拨弹始终无输出，也不保存一次待发请求。 */
}
void shoot_control_update(ShootController *s, const ShootCommand *c, const ShootInput *in,
                          bool permit, float dt, ShootOutput *out)
{
    memset(out, 0, sizeof(*out));
    bool finite = isfinite(dt) && dt > 0 && dt <= 0.02f && isfinite(in->trigger_angle_deg);
    for (unsigned i = 0; i < 3; i++)
        finite = finite && isfinite(in->rpm[i]);
    if (!permit || !in->valid || !finite || !c->enabled ||
        (unsigned)c->mode > SHOOT_REVERSE)
    {
        shoot_control_reset(s);
        return;
    }
    if (!c->friction || c->mode == SHOOT_STOP)
    {
        brake_update(s, in, dt, out);
        return;
    }
    if (s->braking)
        shoot_control_reset(s); /* 刹车途中重新打开，也要重新松扳机、重新到速。 */
    /* 断线恢复时若仍按住扳机，只等待松开，不能补射。 */
    if (!s->armed)
    {
        s->last_fire = c->fire;
        if (!c->fire)
        {
            s->armed = true;
            s->target_angle = in->trigger_angle_deg;
        }
        return;
    }
    out->armed = true;
    s->friction_running = true;
    s->friction_rpm = ramp_update(s->friction_rpm, FRICTION_RPM, FRICTION_ACCEL_RPM_S, dt);
    out->target_rpm[0] = FRICTION_LEFT_DIRECTION * s->friction_rpm;
    out->target_rpm[1] = FRICTION_RIGHT_DIRECTION * s->friction_rpm;
    bool at_speed = s->friction_rpm >= FRICTION_RPM - 1 &&
                    fabsf(in->rpm[0] - FRICTION_LEFT_DIRECTION * FRICTION_RPM) < FRICTION_READY_ERROR_RPM &&
                    fabsf(in->rpm[1] - FRICTION_RIGHT_DIRECTION * FRICTION_RPM) < FRICTION_READY_ERROR_RPM;
    s->ready_s = at_speed ? s->ready_s + dt : 0;
    out->friction_ready = s->ready_s >= FRICTION_READY_TIME_S;
    if (c->mode != s->last_mode)
    {
        s->target_angle = in->trigger_angle_deg;
        pid_clear(&s->speed[2]);
        pid_clear(&s->angle);
        s->last_mode = c->mode;
    }
    bool edge = c->fire && !s->last_fire;
    s->last_fire = c->fire;
    float trigger = 0;
    if (!out->friction_ready)
    {
        /* 摩擦轮没到速时不保存待发事件。再次按下才会单发。 */
        s->target_angle = in->trigger_angle_deg;
        pid_clear(&s->speed[2]);
        pid_clear(&s->angle);
        s->jam_s = 0;
        if (s->reverse_s > 0)
            s->jam_latched = true; /* 退弹途中掉速，也不能自动开始下一次重试。 */
        s->reverse_s = 0;
    }
    else if (s->reverse_s > 0)
    {
        trigger = -TRIGGER_REVERSE_RPM;
        s->reverse_s = fmaxf(0, s->reverse_s - dt);
        s->target_angle = in->trigger_angle_deg;
        out->reversing = true;
        if (s->reverse_s == 0)
            s->jam_latched = true; /* 一次退弹后等松开扳机，避免无限正反转。 */
    }
    else if (!s->jam_latched)
    {
        if (c->mode == SHOOT_CONTINUOUS)
            trigger = c->fire ? TRIGGER_CONTINUOUS_RPM : 0;
        else if (c->mode == SHOOT_REVERSE)
            trigger = c->fire ? -TRIGGER_REVERSE_RPM : 0;
        else
        {
            if (edge && fabsf(s->target_angle - in->trigger_angle_deg) < 90)
                s->target_angle += TRIGGER_STEP_DEG * (c->mode == SHOOT_BURST3 ? 3 : 1);
            trigger = pid_calculate(&s->angle, in->trigger_angle_deg, s->target_angle, dt);
        }
        /* 持续大电流且不转才认为卡住，不能把一次启动电流尖峰当堵弹。 */
        bool jam = fabsf(trigger) > 100 && fabsf(in->rpm[2]) < 100 &&
                   abs(in->trigger_current_raw) >= TRIGGER_JAM_CURRENT;
        s->jam_s = jam ? s->jam_s + dt : 0;
        if (s->jam_s >= TRIGGER_JAM_TIME_S)
        {
            s->reverse_s = TRIGGER_REVERSE_TIME_S;
            s->jam_s = 0;
            trigger = 0;
        }
    }
    else
    {
        s->target_angle = in->trigger_angle_deg;
        if (!c->fire)
            s->jam_latched = false;
    }
    out->target_rpm[2] = trigger;
    for (unsigned i = 0; i < 2; i++)
        out->raw[i] = (int16_t)pid_calculate(&s->speed[i], in->rpm[i], out->target_rpm[i], dt);
    if (out->friction_ready && !s->jam_latched)
        out->raw[2] = (int16_t)pid_calculate(&s->speed[2], in->rpm[2], trigger, dt);
    else
        pid_clear(&s->speed[2]);
    out->jam_latched = s->jam_latched;
}
