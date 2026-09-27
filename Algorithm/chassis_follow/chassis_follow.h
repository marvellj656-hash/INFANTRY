#ifndef CHASSIS_FOLLOW_H
#define CHASSIS_FOLLOW_H
#include "pid.h"
typedef struct { pid_obj_t pid; } ChassisFollow;
typedef struct {
    float error_deg, wz_rad_s;
    bool active;
} ChassisFollowOutput;
void chassis_follow_init(ChassisFollow *s, const pid_config_t *pid);
/* relative_deg 是云台相对车头角度，不是 INS 的世界坐标 Yaw。
 * 正相对角产生正车体旋转；实车符号由调用层配置。纯计算，不访问电机。 */
void chassis_follow_update(ChassisFollow *s, float relative_deg, float deadband_deg,
                           bool permit, float dt, ChassisFollowOutput *out);
#endif
