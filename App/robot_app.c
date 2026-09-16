#include "robot_app.h"
#include "motor_feedback.h"
#include "bsp_motor_can.h"
#include "chassis_config.h"
#include "bsp_remote_uart.h"
#include "dbus_protocol.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "remote/dbus_protocol.h"
#define RC_TIMEOUT pdMS_TO_TICKS(100)
typedef struct {
    PlanarVelocity target;
    float yaw_rate;
    bool neutral;
    TickType_t received_at;
    bool enabled;
} VelocityCommand;//send to controltask package
static QueueHandle_t command_queue; //two tasks queue instead of sub and pub
volatile RemoteDebug debug_remote;//remote for debug
volatile ControlDebug debug_control;
volatile ChassisOutput debug_chassis;
/* Compatibility 兼容with the previous one-axis teaching watches. */
volatile float debug_target_mm_s, debug_ramped_mm_s;
volatile uint32_t debug_control_count, debug_cmd_timeout;

static void RemoteTask(void *argument)
{
    (void)argument;//避免 unused parameter 警告
    DbusState state = {0};
    bool valid = false;
    TickType_t received_at = 0;//最近一帧有效遥控器数据是什么时候收到
    TickType_t wake = xTaskGetTickCount();//record the time base fo delayuntil
    for (;;) {
        uint32_t restarts_before = debug_remote_rx.restart_attempts;//UART 一共重启过多少次
        bool transport_ok = bsp_remote_uart_service();
        if (!transport_ok || debug_remote_rx.restart_attempts != restarts_before)//didn't recover or had restart
            valid = false; /* A restarted receiver needs a new valid frame. */
        RemotePacket packet;//to accomodate the remote
        if (bsp_remote_uart_take(&packet)) {//packet has the received at
            valid = packet.valid_length &&
                    dbus_decode(packet.data, DBUS_FRAME_BYTES, &state);
            if (valid) {
                received_at = packet.received_at;
                debug_remote.valid_frames++;
            } else debug_remote.invalid_frames++;
        }
        TickType_t now = xTaskGetTickCount();
        TickType_t age = now - received_at;
        bool online = transport_ok && valid && age < RC_TIMEOUT;
        /* Phase 1: only manual (sw2 middle=3). Auto/down remains disabled. */
        bool enabled = online && state.sw2 == 3;//遥控在线 + SW2中档；不要求云台归中
        ChassisVelocity requested = chassis_target(state.ch[0], state.ch[1], state.ch[2]);
        VelocityCommand command = {
            .target = {requested.vx, requested.vy},
            .yaw_rate = requested.wz,
            .neutral = requested.vx == 0 && requested.vy == 0 && requested.wz == 0,
            .received_at = received_at,
            .enabled = enabled
        };
        debug_remote.ch1 = online ? state.ch[0] : 0;
        debug_remote.ch2 = online ? state.ch[1] : 0;
        debug_remote.ch3 = online ? state.ch[2] : 0;
        debug_remote.ch4 = online ? state.ch[3] : 0;
        debug_remote.sw1 = online ? state.sw1 : 1;
        debug_remote.sw2 = online ? state.sw2 : 1;//disabled
        debug_remote.online = online;
        debug_remote.enabled = enabled;
        debug_remote.age_ms = valid ?
            (uint32_t)((uint64_t)age * 1000u / configTICK_RATE_HZ) : UINT32_MAX;//to ms
        xQueueOverwrite(command_queue, &command);//send it
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));//按10ms计划周期等待；高优先级任务或中断仍可能推迟实际执行
    }
}

static void ControlTask(void *argument)
{
    (void)argument;
    TickType_t wake = xTaskGetTickCount(), previous = wake;
    VelocityCommand command = {0};
    ChassisController controller = {0};
    bool previous_tx_ok = true;
    for (;;) {
        TickType_t now = xTaskGetTickCount();
        TickType_t elapsed = now - previous;
        previous = now;
        (void)xQueueReceive(command_queue, &command, 0);
        bool fresh = (TickType_t)(now - command.received_at) < RC_TIMEOUT;
        bool timing_ok = elapsed > 0 && elapsed <= pdMS_TO_TICKS(20);
        if (elapsed > pdMS_TO_TICKS(20)) {
            debug_control.timing_faults++;
            wake = now; /* 超期时重建周期基准，不连续补跑历史控制周期。 */
        }

        /* 读取任务发布的完整快照，不能把 LinkScope 调试结构当并发控制接口。 */
        ChassisFeedback feedback;
        motor_feedback_get_chassis(&feedback);
        float measured[4];
        bool motors_ready = feedback.valid_mask == 0x0fu;
        for (unsigned i = 0; i < 4; ++i) {
            measured[i] = feedback.speed_rpm[i];
            motors_ready = motors_ready &&
                (TickType_t)(now - feedback.received_at[i]) < pdMS_TO_TICKS(20);
        }
        bool can_ready = bsp_motor_can_ready();
        uint32_t faults = 0;
        if (!command.enabled || !fresh) faults |= CHASSIS_FAULT_REMOTE;
        if (!motors_ready) faults |= CHASSIS_FAULT_MOTOR;
        if (!timing_ok) faults |= CHASSIS_FAULT_TIMING;
        if (!can_ready || !previous_tx_ok) faults |= CHASSIS_FAULT_CAN;
        ChassisVelocity target = {command.target.vx, command.target.vy, command.yaw_rate};
        ChassisOutput result;
        chassis_step(&controller, target, measured, faults == 0, command.neutral,
                     (float)elapsed / configTICK_RATE_HZ,
                     CHASSIS_BRINGUP_POWER_W, &result);
        if (faults == 0 && !result.armed) faults |= CHASSIS_WAIT_NEUTRAL;

        /* 唯一的底盘电流发送入口，每 2 ms 一帧 0x200。
         * 停机不只是停止发送，而是发送四路零电流并清 PID/斜坡状态。
         * 不读取云台角度或 BACK_IS_OK；本阶段使用底盘自身坐标系。
         */
        previous_tx_ok = bsp_motor_can_send_chassis(result.current_raw);
        if (!previous_tx_ok) {
            faults |= CHASSIS_FAULT_CAN;
            chassis_reset(&controller);
            result.armed = 0;
        }
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
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(2));
    }
}


bool robot_app_init(void)
{
    if (!bsp_remote_uart_init()) return false;
    if (!motor_feedback_init()) return false;
    command_queue = xQueueCreate(1, sizeof(VelocityCommand));
    if (!command_queue) return false;
    if (xTaskCreate(RemoteTask, "remote", 512, NULL, 3, NULL) != pdPASS)
        return false;
    return xTaskCreate(ControlTask, "control", 512, NULL, 4, NULL) == pdPASS;
}
