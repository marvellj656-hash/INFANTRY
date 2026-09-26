#include "omni_kinematics.h"
#include <math.h>
/* 车体速度 -> 轮缘速度 -> 减速器输入转子 RPM。 */
void omni_inverse_kinematics(ChassisVelocity v, const OmniGeometry *geometry, float rpm[4])
{
    const float k = 60.0f * geometry->gear_ratio / geometry->wheel_perimeter_mm;
    const float x = 0.70710678f * v.vx;
    const float y = 0.70710678f * v.vy;
    const float rotate = v.wz * geometry->radius_mm;
    rpm[0] = ( x + y + rotate) * k;
    rpm[1] = ( x - y + rotate) * k;
    rpm[2] = (-x - y + rotate) * k;
    rpm[3] = (-x + y + rotate) * k;
    float peak = 0;
    for (unsigned i = 0; i < 4; ++i) peak = fmaxf(peak, fabsf(rpm[i]));
    if (peak > geometry->max_motor_rpm)
        for (unsigned i = 0; i < 4; ++i) rpm[i] *= geometry->max_motor_rpm / peak;
}


ChassisVelocity omni_forward_kinematics(const float rpm[4], const OmniGeometry *geometry)
{
    /* k 的单位是 RPM/(mm/s)。原 RTT 的 omni_get_speed 在这里混入了 rad/s，
     * 移植后输入明确为电机反馈 speed_rpm，与逆解使用同一个减速比。 */
    const float k = 60.0f * geometry->gear_ratio / geometry->wheel_perimeter_mm;
    ChassisVelocity measured;
    measured.vx = (rpm[0] + rpm[1] - rpm[2] - rpm[3]) / (4.0f * 0.70710678f * k);
    measured.vy = (rpm[0] - rpm[1] - rpm[2] + rpm[3]) / (4.0f * 0.70710678f * k);
    measured.wz = (rpm[0] + rpm[1] + rpm[2] + rpm[3]) / (4.0f * geometry->radius_mm * k);
    return measured;
}
