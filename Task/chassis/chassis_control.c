#include "chassis_control.h"
#include "chassis_config.h"
#include "ramp.h"
#include "power_model.h"
#include <math.h>
#include <string.h>
/* 应用决定几何、调参和预算；算法只接收参数，不认识整车配置。 */
static const OmniGeometry geometry = {CHASSIS_WHEEL_PERIMETER_MM, CHASSIS_GEAR_RATIO,
                                      CHASSIS_RADIUS_MM, CHASSIS_MAX_MOTOR_RPM};
static const pid_config_t speed_pid[4] = CHASSIS_SPEED_PID_CONFIGS;
static const PowerModel power_model = {POWER_RAW_TO_A, POWER_ROTOR_KT, POWER_EQUIVALENT_R,
                                       POWER_IDLE_W};
static float clamp(float x, float limit)
{
    if (x > limit)
        return limit;
    if (x < -limit)
        return -limit;
    return x;
}
void chassis_control_reset(ChassisController *controller)
{
    memset(controller, 0, sizeof(*controller));
    for (unsigned i = 0; i < 4; ++i)
        pid_init(&controller->pid[i], &speed_pid[i]);
}
void chassis_control_update(ChassisController *s, ChassisVelocity target, const float rpm[4],
                            bool permit, bool neutral, float dt, float budget, ChassisOutput *out)
{
    memset(out, 0, sizeof(*out));
    out->budget_w = budget;
    bool feedback_finite = true;
    bool finite = isfinite(target.vx) && isfinite(target.vy) && isfinite(target.wz); //
    for (unsigned i = 0; i < 4; ++i)
    {
        feedback_finite = feedback_finite && isfinite(rpm[i]);
        finite = finite && isfinite(rpm[i]);
        out->measured_rpm[i] = rpm[i];
    }
    /* 停机时车仍可能被推着走，反馈解算不依赖遥控是否使能。
     * 这里只检查数值；任务层再用反馈时间戳判断是否新鲜。 */
    if (feedback_finite)
    {
        out->measured_velocity = omni_forward_kinematics(rpm, &geometry);
        out->measured_velocity_valid = 1;
    }
    if (!permit || !finite || !isfinite(dt) || dt <= 0 || dt > 0.02f || !isfinite(budget) ||
        budget <= POWER_IDLE_W)
    {
        chassis_control_reset(s);
        return;
    }
    /* 防止开机/失联恢复时，摇杆仍推着就突然动；这是摇杆零位，不是云台条件。 */
    if (!s->armed)
    {
        if (neutral)
            s->armed = true;
        out->armed = s->armed;
        return;
    }

    /* 合速度限幅：斜向不会比单轴快；上限由 chassis_config.h 决定。 */
    float norm = hypotf(target.vx, target.vy);
    if (norm > CHASSIS_MAX_SPEED_MM_S)
    {
        target.vx *= CHASSIS_MAX_SPEED_MM_S / norm;
        target.vy *= CHASSIS_MAX_SPEED_MM_S / norm;
    }
    target.wz = clamp(target.wz, CHASSIS_MAX_YAW_RAD_S);
    out->target = target;
    ramp_update_xy(&s->velocity.vx, &s->velocity.vy, target.vx, target.vy, CHASSIS_ACCEL_MM_S2, dt);
    s->velocity.wz = target.wz; /* RTT旋转目标直接进运动学，无额外旋转斜坡。 */
    out->velocity = s->velocity;
    out->armed = 1;
    omni_inverse_kinematics(s->velocity, &geometry, out->target_rpm);
    out->feasible_velocity = omni_forward_kinematics(out->target_rpm, &geometry);

    float raw[4], old_i[4];
    for (unsigned i = 0; i < 4; ++i)
    {
        old_i[i] = s->pid[i].Iout;
        raw[i] = pid_calculate(&s->pid[i], rpm[i], out->target_rpm[i], dt);
    }
    out->power_scale = power_model_limit(&power_model, raw, rpm, budget, &out->estimated_power_w);
    for (unsigned i = 0; i < 4; ++i)
    {
        /* 后级功率限制也属于饱和：回退本周期积分，避免解除功限后突然冲击。 */
        if (out->power_scale < 0.999f && (out->target_rpm[i] - rpm[i]) * raw[i] >
                                             0) // powerlimit trigered so Pi should anti-windup
            s->pid[i].Iout = old_i[i];
        out->current_raw[i] = (int16_t)raw[i];
    }
}
