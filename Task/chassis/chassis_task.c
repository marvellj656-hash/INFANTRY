/* 任务负责取快照、检查许可、调用纯计算、提交输出。 */
#ifndef CHASSIS_HOST_TEST
#include "chassis_task.h"
#include "chassis_config.h"
#include "motor/dji_motor.h"
#include "gimbal/gimbal_task.h"
#include "robot_config.h"
#include "bsp_time.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
static QueueHandle_t command_queue;
static DjiMotorHandle wheels[4];
/* 控制器本体：四轮 PID 和斜坡状态直接放这里，不另建调试副本。
 * 只有底盘任务写入；LinkScope 可查看 chassis_controller.pid[0].Iout。 */
static ChassisController chassis_controller;
static ChassisFollow follow_controller;
volatile ChassisFollowOutput debug_follow;
volatile ChassisFollowConfigDebug debug_follow_config;
volatile ControlDebug debug_control;
volatile ChassisOutput debug_chassis;
volatile float debug_target_mm_s, debug_ramped_mm_s;
volatile uint32_t debug_control_count, debug_cmd_timeout;
bool chassis_init(void)
{
    const DjiMotorId ids[4] = {MOTOR_CHASSIS_FL, MOTOR_CHASSIS_FR, MOTOR_CHASSIS_RL,
                               MOTOR_CHASSIS_RR};
    for (unsigned i = 0; i < 4; ++i)
    {
        wheels[i] = dji_motor_find(ids[i]);
        if (!wheels[i])
            return false;
    }
    command_queue = xQueueCreate(1, sizeof(ChassisCommand));
    return command_queue != NULL;
}
bool chassis_set_command(const ChassisCommand *command)
{
    /* 长度为 1 的队列只保留最新指令，队列句柄不暴露给 cmd_task。 */
    return command_queue && command && xQueueOverwrite(command_queue, command) == pdPASS;
}
void chassis_task_entry(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount(), previous = wake;
    ChassisCommand command = {0};
    chassis_control_reset(&chassis_controller);
    const pid_config_t follow_pid = CHASSIS_FOLLOW_PID_CONFIG;
    debug_follow_config = (ChassisFollowConfigDebug){
        .deadband_deg = CHASSIS_FOLLOW_DEADBAND_DEG,
        .yaw_accel_rad_s2 = CHASSIS_YAW_ACCEL_RAD_S2};
    chassis_follow_init(&follow_controller, &follow_pid);
    uint32_t reference_generation = 0;
    bool previous_tx_ok = true;
    for (;;)
    {
        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = now - previous;
        previous = now;
        (void)xQueueReceive(command_queue, &command, 0);
        uint32_t now_ms = bsp_time_ms();
        bool fresh = (uint32_t)(now_ms - command.received_at_ms) < ROBOT_REMOTE_TIMEOUT_MS;
        bool timing_ok = elapsed > 0 && elapsed <= pdMS_TO_TICKS(ROBOT_CONTROL_MAX_INTERVAL_MS);
        if (elapsed > pdMS_TO_TICKS(ROBOT_CONTROL_MAX_INTERVAL_MS))
        {
            wake = now; /* 超期时重建周期基准，不连续补跑历史控制周期。 */
        }

        /* 读取任务发布的完整快照，不能把 LinkScope 调试结构当并发控制接口。 */
        DjiMotorFeedback feedback[4] = {0};
        bool motors_ready = dji_motor_read_many(wheels, 4, feedback);
        bool can_ready = motors_ready;
        float measured[4];
        for (unsigned i = 0; i < 4; ++i)
        {
            measured[i] = feedback[i].measure.speed_rpm;
            motors_ready = motors_ready && feedback[i].online;
            can_ready = can_ready && feedback[i].transport_ready;
        }
        uint32_t faults = 0;
        if (!command.enabled || !fresh)
            faults |= CHASSIS_FAULT_REMOTE;
        if (!motors_ready)
            faults |= CHASSIS_FAULT_MOTOR;
        if (!timing_ok)
            faults |= CHASSIS_FAULT_TIMING;
        if (!can_ready || !previous_tx_ok)
            faults |= CHASSIS_FAULT_CAN;
        ChassisVelocity target = command.target;
        GimbalOutput gimbal = {0};
        bool reference_ok = gimbal_get_state(&gimbal) && gimbal.centered && gimbal.reference_valid;
        if (reference_generation != gimbal.reference_generation)
        {
            reference_generation = gimbal.reference_generation;
            pid_clear(&follow_controller.pid);
        }
        ChassisFollowOutput follow;
        chassis_follow_update(&follow_controller,
            CHASSIS_FOLLOW_DIRECTION * gimbal.relative_deg[0], CHASSIS_FOLLOW_DEADBAND_DEG,
            command.follow_gimbal && reference_ok && faults == 0,
            (float)elapsed / configTICK_RATE_HZ, &follow);
        if (command.follow_gimbal)
        {
            target.wz = follow.wz_rad_s;
            if (!follow.active)
                chassis_controller.velocity.wz = 0; /* 参考失效，清掉旧跟随斜坡。 */
        }
        debug_follow = follow;
        ChassisOutput result;
        chassis_control_update(&chassis_controller, target, measured, faults == 0, command.neutral,
                               (float)elapsed / configTICK_RATE_HZ, CHASSIS_BRINGUP_POWER_W,
                               &result);
        result.measured_velocity_valid = result.measured_velocity_valid && motors_ready;
        if (!result.measured_velocity_valid)
            result.measured_velocity = (ChassisVelocity){0}; /* 不把过期速度当成当前车速。 */
        if (faults == 0 && !result.armed)
            faults |= CHASSIS_WAIT_NEUTRAL;

        /* 四轮一起提交，电机服务统一发送。停机仍提交零电流并清 PID。
         * 返回值只表示提交成功；实际 CAN 状态由下一轮快照报告。
         * motor_task 独立检查指令超时：控制任务停止刷新时也会输出零。 */
        previous_tx_ok = dji_motor_set_outputs(wheels, result.current_raw, 4);
        if (!previous_tx_ok)
        {
            faults |= CHASSIS_FAULT_CAN;
            chassis_control_reset(&chassis_controller);
            result.armed = 0;
        }
        /* 2 ms 的故障可能在 LinkScope 下一次采样前消失，留下计数便于定位。
         * 这里只记录原因，不改变原来的停机和归中规则。 */
        uint32_t actual_faults = faults & (CHASSIS_FAULT_REMOTE | CHASSIS_FAULT_MOTOR |
                                           CHASSIS_FAULT_TIMING | CHASSIS_FAULT_CAN);
        if (actual_faults)
            debug_control.last_fault_flags = actual_faults;
        if (faults & CHASSIS_FAULT_REMOTE)
            debug_control.remote_fault_cycles++;
        if (faults & CHASSIS_FAULT_MOTOR)
            debug_control.motor_fault_cycles++;
        if (faults & CHASSIS_FAULT_TIMING)
            debug_control.timing_faults++;
        if (faults & CHASSIS_FAULT_CAN)
            debug_control.can_fault_cycles++;
        result.fault_flags = faults;
        debug_chassis = result;
        debug_control.target = (PlanarVelocity){result.target.vx, result.target.vy};
        debug_control.output = (PlanarVelocity){result.velocity.vx, result.velocity.vy};
        debug_control.enabled = result.armed && faults == 0;
        debug_control.cycles++;
        debug_target_mm_s = result.target.vx;
        debug_ramped_mm_s = result.velocity.vx;
        debug_control_count++;
        debug_cmd_timeout = !fresh;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(ROBOT_CHASSIS_PERIOD_MS));
    }
}

#endif
