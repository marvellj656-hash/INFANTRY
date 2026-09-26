#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#include "omni_kinematics.h"
#include "pid.h"
typedef struct
{
    ChassisVelocity velocity;
    pid_obj_t pid[4];
    bool armed;
} ChassisController;
typedef struct
{
    ChassisVelocity target, velocity;
    float target_rpm[4], measured_rpm[4];
    int16_t current_raw[4];
    float power_scale, estimated_power_w, budget_w;
    uint32_t armed, fault_flags;
    ChassisVelocity measured_velocity; /* 四轮反馈解算的车体速度，mm/s、rad/s。 */
    uint32_t measured_velocity_valid;  /* 任意轮反馈失效时不能当实际速度使用。 */
} ChassisOutput;
enum
{
    CHASSIS_FAULT_REMOTE = 1u,
    CHASSIS_FAULT_MOTOR = 2u,
    CHASSIS_FAULT_TIMING = 4u,
    CHASSIS_FAULT_CAN = 8u,
    CHASSIS_WAIT_NEUTRAL = 16u
};
void chassis_control_reset(ChassisController *controller);
/* 纯算法：无 HAL、无任务/动态分配。permit=false 时立即零电流并清积分。
 * 首次使能或故障恢复后，要求本地遥控指令归零一次，不要求云台归中。
 */
void chassis_control_update(ChassisController *controller, ChassisVelocity target,
                            const float measured_rpm[4], bool permit, bool neutral, float dt,
                            float budget_w, ChassisOutput *out);

#endif
