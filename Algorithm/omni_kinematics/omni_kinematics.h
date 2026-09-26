#ifndef ALGORITHM_OMNI_KINEMATICS_H
#define ALGORITHM_OMNI_KINEMATICS_H

typedef struct { float vx, vy, wz; } ChassisVelocity; /* mm/s, mm/s, rad/s */
typedef struct {
    float wheel_perimeter_mm, gear_ratio, radius_mm, max_motor_rpm;
} OmniGeometry;
/* 沿用原工程四轮顺序；参数必须为正。rpm 输出为电机转子转速。 */
void omni_inverse_kinematics(ChassisVelocity velocity, const OmniGeometry *geometry, float rpm[4]);
//omni reverse 运动学
/* 四轮实测转子 RPM -> 车体坐标速度。不是 IMU 航向，也不补偿轮胎打滑。
 * 沿用逆解的轮序和正方向；不能把 speed_rad 传入此接口。 */
ChassisVelocity omni_forward_kinematics(const float rpm[4], const OmniGeometry *geometry);

#endif
