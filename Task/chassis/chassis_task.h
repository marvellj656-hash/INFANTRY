#ifndef APP_CHASSIS_CHASSIS_TASK_H
#define APP_CHASSIS_CHASSIS_TASK_H

#include <stdbool.h>
#include <stdint.h>
#include "omni_kinematics.h"
#include "pid.h"
#include "chassis_control.h"
#include "chassis_follow.h"
typedef struct
{
    ChassisVelocity target;
    uint32_t received_at_ms; /* 保留原始遥控帧时间，不能用任务更新时间冒充新数据。 */
    bool enabled, neutral;
    bool follow_gimbal; /* 只表达模式；相对角由底盘任务读取新鲜云台快照。 */
} ChassisCommand;
typedef struct
{
    float vx, vy;
} PlanarVelocity; /* 仅保留旧 LinkScope 显示结构。 */
typedef struct
{
    PlanarVelocity target, output;
    uint32_t enabled, cycles, timing_faults;
    /* 累计的是故障周期数，不是丢包数；观察摇杆操作前后的增量。
     * last_fault_flags 保留最近真实故障，不被等待归中(16)覆盖。 */
    uint32_t last_fault_flags, remote_fault_cycles, motor_fault_cycles, can_fault_cycles;
} ControlDebug;
extern volatile ControlDebug debug_control;
extern volatile ChassisOutput debug_chassis;
extern volatile ChassisFollowOutput debug_follow;
typedef struct
{
    float deadband_deg, yaw_accel_rad_s2;
} ChassisFollowConfigDebug;
/* 只供 LinkScope 看当前编译参数。控制器不读取此副本，修改它不会调参。 */
extern volatile ChassisFollowConfigDebug debug_follow_config;
bool chassis_init(void); /* 只创建命令队列，不创建任务。 */
bool chassis_set_command(const ChassisCommand *command);
void chassis_task_entry(void *argument);

#endif
