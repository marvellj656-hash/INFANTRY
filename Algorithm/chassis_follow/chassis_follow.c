#include "chassis_follow.h"
#include <math.h>
void chassis_follow_init(ChassisFollow *s, const pid_config_t *pid)
{
    pid_init(&s->pid, pid);
}
void chassis_follow_update(ChassisFollow *s, float relative_deg, float deadband_deg,
                           bool permit, float dt, ChassisFollowOutput *out)
{
    *out = (ChassisFollowOutput){0};
    if (!permit || !isfinite(relative_deg) || !isfinite(deadband_deg) ||
        deadband_deg < 0 || deadband_deg >= 180 || !isfinite(dt) || dt <= 0 || dt > 0.02f)
    {
        pid_clear(&s->pid);
        return;
    }
    out->active = true;
    /* 相对角359度等同于-1度，不能绕一大圈追目标。 */
    out->error_deg = remainderf(relative_deg, 360.0f);
    if (deadband_deg > 0 && fabsf(out->error_deg) <= deadband_deg)
    {
        pid_clear(&s->pid);
        return;
    }
    /* 死区外减掉死区宽度，跨过边界时不会一下跳出很大的旋转速度。 */
    float error = out->error_deg - copysignf(deadband_deg, out->error_deg);
    out->wz_rad_s = pid_calculate(&s->pid, -error, 0, dt);
}
