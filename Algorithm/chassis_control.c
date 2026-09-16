#include "chassis_control.h"
#include "chassis_config.h"
#include <math.h>
#include <string.h>

static float clamp(float x, float limit)
{
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}

void chassis_reset(ChassisController *controller)
{
    memset(controller, 0, sizeof(*controller));
}

static float channel_axis(int16_t channel)
{
    float x = clamp((float)channel, 660);
    if (x > 5) return (x - 5) / 655.0f;
    if (x < -5) return (x + 5) / 655.0f;
    return 0;
}

ChassisVelocity chassis_target(int16_t ch1, int16_t ch2, int16_t ch3)
{
    ChassisVelocity v = {channel_axis(ch1), channel_axis(ch2), channel_axis(ch3)};
    float length = hypotf(v.vx, v.vy);
    if (length > 1) { v.vx /= length; v.vy /= length; }
    /* 在通道映射处使用低速上限，而不是末端硬截断，保留整个摇杆行程。 */
    v.vx *= CHASSIS_MAX_SPEED_MM_S;
    v.vy *= CHASSIS_MAX_SPEED_MM_S;
    v.wz *= CHASSIS_MAX_YAW_RAD_S;
    return v;
}

/* 沿用原 omni_calc 的四轮符号。这里明确 wz 为 rad/s，所以乘半径即 mm/s。
 * wheel_mm_s / 轮周长 -> 轮转/秒；乘 60 和减速比 -> 转子 RPM。
 */
void chassis_inverse(ChassisVelocity v, float rpm[4])
{
    const float k = 60.0f * CHASSIS_GEAR_RATIO / CHASSIS_WHEEL_PERIMETER_MM;
    const float x = 0.70710678f * v.vx;
    const float y = 0.70710678f * v.vy;
    const float rotate = v.wz * CHASSIS_RADIUS_MM;
    rpm[0] = ( x + y + rotate) * k;
    rpm[1] = ( x - y + rotate) * k;
    rpm[2] = (-x - y + rotate) * k;
    rpm[3] = (-x + y + rotate) * k;
    float peak = 0;
    for (unsigned i = 0; i < 4; ++i) peak = fmaxf(peak, fabsf(rpm[i]));
    if (peak > CHASSIS_MAX_MOTOR_RPM)
        for (unsigned i = 0; i < 4; ++i) rpm[i] *= CHASSIS_MAX_MOTOR_RPM / peak;
}

float chassis_power_limit(float raw[4], const float rpm[4], float budget, float *estimated)
{
    /* 对统一缩放系数 s：P(s)=A*s²+B*s+C。
     * A 为电阻损耗 ΣR*I²，B 为 Σ|Kt*I*ω|，C 为固定损耗估计。
     * 再生制动的负机械功率不拿来抵扣其他轮耗电，先采用保守上界。
     * 与旧代码不同：ω 来自实测 RPM 转 rad/s，不混用 motor_ref 和角速度。
     */
    float a = 0, b = 0;
    bool valid = isfinite(budget) && budget >= 0;
    for (unsigned i = 0; i < 4; ++i) {
        valid = valid && isfinite(raw[i]) && isfinite(rpm[i]);
        float amp = raw[i] * POWER_RAW_TO_A;
        a += POWER_EQUIVALENT_R * amp * amp;
        b += fabsf(POWER_ROTOR_KT * amp * rpm[i] * 0.104719755f);
    }
    float scale = 0;
    if (valid && isfinite(a) && isfinite(b) && budget > POWER_IDLE_W) {
        float available = budget - POWER_IDLE_W;
        if (a + b <= available) scale = 1;
        else if (a > 0.000001f) {
            /* 正根的等价写法，避免 -B + sqrt(...) 相消导致精度损失。 */
            scale = 2.0f * available / (b + sqrtf(b*b + 4.0f*a*available));
        } else if (b > 0) scale = available / b;
    }
    scale = fmaxf(0, fminf(scale, 1));
    for (unsigned i = 0; i < 4; ++i) raw[i] = valid ? raw[i] * scale : 0;
    *estimated = valid && isfinite(a) && isfinite(b) ?
        a*scale*scale + b*scale + POWER_IDLE_W : 0;
    return scale;
}

void chassis_step(ChassisController *s, ChassisVelocity target,
                  const float rpm[4], bool permit, bool neutral,
                  float dt, float budget, ChassisOutput *out)
{
    memset(out, 0, sizeof(*out));
    out->budget_w = budget;
    bool finite = isfinite(target.vx) && isfinite(target.vy) && isfinite(target.wz);
    for (unsigned i = 0; i < 4; ++i) {
        finite = finite && isfinite(rpm[i]);
        out->measured_rpm[i] = rpm[i];
    }
    if (!permit || !finite || !isfinite(dt) || dt <= 0 || dt > 0.02f ||
        !isfinite(budget) || budget <= POWER_IDLE_W) {
        chassis_reset(s);
        return;
    }
    /* 防止开机/失联恢复时，摇杆仍推着就突然动；这是摇杆零位，不是云台条件。 */
    if (!s->armed) {
        if (neutral) s->armed = true;
        out->armed = s->armed;
        return;
    }

    /* 合速度限幅：斜向不会比单轴快；首次试车只允许 300 mm/s。 */
    float norm = hypotf(target.vx, target.vy);
    if (norm > CHASSIS_MAX_SPEED_MM_S) {
        target.vx *= CHASSIS_MAX_SPEED_MM_S / norm;
        target.vy *= CHASSIS_MAX_SPEED_MM_S / norm;
    }
    target.wz = clamp(target.wz, CHASSIS_MAX_YAW_RAD_S);
    out->target = target;
    float dx = target.vx - s->velocity.vx;
    float dy = target.vy - s->velocity.vy;
    float distance = hypotf(dx, dy);
    float step = CHASSIS_ACCEL_MM_S2 * dt;
    float ratio = distance > step ? step / distance : 1;
    s->velocity.vx += dx * ratio;
    s->velocity.vy += dy * ratio;
    s->velocity.wz += clamp(target.wz - s->velocity.wz, CHASSIS_YAW_ACCEL_RAD_S2 * dt);
    out->velocity = s->velocity;
    out->armed = 1;
    chassis_inverse(s->velocity, out->target_rpm);

    float raw[4], old_i[4];
    for (unsigned i = 0; i < 4; ++i) {
        ChassisPid *pid = &s->pid[i];
        float error = out->target_rpm[i] - rpm[i];
        old_i[i] = pid->integral;
        if (!pid->initialized) {
            pid->previous_error = error;
            pid->previous_measure = rpm[i];
            pid->initialized = true;
        }
        /* 梯形积分 + 对测量值求微分，沿用原速度 PID 的含义和系数。 */
        float proposed_i = clamp(pid->integral + CHASSIS_KI *
                                  (error + pid->previous_error) * 0.5f * dt, CHASSIS_I_LIMIT);
        float pd = CHASSIS_KP * error - CHASSIS_KD * (rpm[i] - pid->previous_measure) / dt;
        /* 输出饱和且误差还在推动饱和时，不继续积累积分。 */
        if (fabsf(pd + proposed_i) <= CHASSIS_CURRENT_LIMIT_RAW || (pd + proposed_i)*error < 0)
            pid->integral = proposed_i;
        raw[i] = clamp(pd + pid->integral, CHASSIS_CURRENT_LIMIT_RAW);
        pid->previous_error = error;
        pid->previous_measure = rpm[i];
    }
    out->power_scale = chassis_power_limit(raw, rpm, budget, &out->estimated_power_w);
    for (unsigned i = 0; i < 4; ++i) {
        /* 后级功率限制也属于饱和：回退本周期积分，避免解除功限后突然冲击。 */
        if (out->power_scale < 0.999f && (out->target_rpm[i] - rpm[i])*raw[i] > 0)
            s->pid[i].integral = old_i[i];
        out->current_raw[i] = (int16_t)raw[i];
    }
}

void chassis_pack_current(const int16_t raw[4], uint8_t bytes[8])
{
    /* CAN 标准数据帧 0x200，一帧按顺序控制 C620 ID1..4，都是高字节在前。 */
    for (unsigned i = 0; i < 4; ++i) {
        int32_t limited = raw[i];
        if (limited > CHASSIS_CURRENT_LIMIT_RAW) limited = CHASSIS_CURRENT_LIMIT_RAW;
        if (limited < -CHASSIS_CURRENT_LIMIT_RAW) limited = -CHASSIS_CURRENT_LIMIT_RAW;
        uint16_t word = (uint16_t)(int16_t)limited;
        bytes[2*i] = (uint8_t)(word >> 8);
        bytes[2*i+1] = (uint8_t)word;
    }
}
