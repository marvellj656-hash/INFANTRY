#include "attitude.h"
#include "QuaternionEKF.h"
#include <string.h>
#include <math.h>
void attitude_reset(AttitudeEstimator *s) { memset(s, 0, sizeof(*s)); }
bool attitude_update(AttitudeEstimator *s, const float g[3], const float a[3], float dt,
                     AttitudeResult *out)
{
    *out = (AttitudeResult){0};
    bool finite = isfinite(dt) && dt > 0 && dt <= 0.02f;
    float norm = 0, rate = 0;
    for (unsigned i = 0; i < 3; i++)
    {
        finite = finite && isfinite(g[i]) && isfinite(a[i]);
        norm += a[i] * a[i];
        rate += g[i] * g[i];
    }
    if (!finite || norm < 1.0f)
    {
        attitude_reset(s);
        return false;
    }
    if (!s->initialized)
    {
        /* 开机先静止约 1 秒，取零偏。晃动时从头计数，不拿运动速度当零偏。 */
        if (fabsf(sqrtf(norm) - 9.80665f) > 0.8f || sqrtf(rate) > 0.08f)
        {
            attitude_reset(s);
            return false;
        }
        for (unsigned i = 0; i < 3; i++)
        {
            s->gyro_sum[i] += g[i];
            s->accel_sum[i] += a[i];
        }
        if (++s->stationary_samples < 500)
            return false;
        for (unsigned i = 0; i < 3; i++)
            s->bias[i] = s->gyro_sum[i] / 500.0f;
        float roll = atan2f(s->accel_sum[1], s->accel_sum[2]);
        float pitch = atan2f(-s->accel_sum[0], hypotf(s->accel_sum[1], s->accel_sum[2]));
        float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f), cp = cosf(pitch * 0.5f),
              sp = sinf(pitch * 0.5f);
        float q[4] = {cr * cp, sr * cp, cr * sp, -sr * sp};
        IMU_QuaternionEKF_Init(q, 10, 0.001f, 1000000, 1, 0);
        s->initialized = true;
    }
    IMU_QuaternionEKF_Update(g[0] - s->bias[0], g[1] - s->bias[1], g[2] - s->bias[2], a[0], a[1],
                             a[2], dt);
    /* 保留旧工程安装映射。旧 EKF 的 Roll 字段实际是绕 Y 轴俯仰角。 */
    *out = (AttitudeResult){-QEKF_INS.YawTotalAngle, QEKF_INS.Roll, -(g[2] - s->bias[2]),
                            g[1] - s->bias[1] - QEKF_INS.GyroBias[1], true};
    if (!isfinite(out->yaw_deg) || !isfinite(out->pitch_deg))
    {
        attitude_reset(s);
        *out = (AttitudeResult){0};
        return false;
    }
    return true;
}
