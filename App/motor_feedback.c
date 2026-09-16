#include "motor_feedback.h"
#include "bsp_motor_can.h"
#include "task.h"

#define MOTOR_TIMEOUT pdMS_TO_TICKS(100)
/* LinkScope 可直接读取这些全局变量。volatile 便于观察，并不提供互斥保护。 */
volatile MotorFeedbackDebug debug_motor[DJI_FEEDBACK_MOTORS];
volatile uint32_t debug_motor_task_cycles;

static ChassisFeedback published_chassis;
void motor_feedback_get_chassis(ChassisFeedback *out)
{
    /* 与下面的发布操作配对，复制约 28 字节，临界区不放等待操作。 */
    taskENTER_CRITICAL();
    *out = published_chassis;
    taskEXIT_CRITICAL();
}

static void MotorFeedbackTask(void *argument)
{
    (void)argument;
    ChassisFeedback working_chassis = {0};
    /* 每个电机分别记时间；CAN1 有报文不等于所有电机都在线。 */
    TickType_t last_rx[DJI_FEEDBACK_MOTORS] = {0};
    bool received[DJI_FEEDBACK_MOTORS] = {false};
    for (unsigned i = 0; i < DJI_FEEDBACK_MOTORS; ++i) {
        debug_motor[i].bus = i == 5 ? 2 : 1;
        debug_motor[i].id = i < 4 ? 0x201u + i : (i == 4 ? 0x207 : 0x208);
        debug_motor[i].age_ms = UINT32_MAX; /* 还没收到过有效帧。 */
    }
    /* 当前仅启动 CAN1。CAN2/Pitch 的调试位置保留，但 online 一直为 0。
     * 启动失败可看 debug_motor_can.start_failures[0]，不阻断遥控器任务。
     */
    (void)bsp_motor_can_start();
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        MotorCanPacket packet;
        /* 每 1 ms 最多处理 32 帧，避免繁忙总线让任务无限占用 CPU。 */
        for (unsigned n = 0; n < 32 && bsp_motor_can_take(&packet); ++n) {
            int index = dji_feedback_index(packet.bus, packet.id);
            if (index < 0) continue; /* 非本工程电机的 ID，忽略。 */
            DjiFeedback value;
            if (!dji_feedback_decode(packet.data, packet.length, &value)) {
                debug_motor[index].invalid_frames++;
                continue;
            }
            debug_motor[index].ecd = value.ecd;
            debug_motor[index].speed_rpm = value.speed_rpm;
            debug_motor[index].current_raw = value.current_raw;
            debug_motor[index].temperature = value.temperature;
            debug_motor[index].frames++;
            /* 用中断收到帧的时刻，不用任务此时处理帧的时刻，避免旧数据变新。 */
            last_rx[index] = packet.received_at;
            received[index] = true;
            if (index < 4) {
                working_chassis.speed_rpm[index] = value.speed_rpm;
                working_chassis.received_at[index] = packet.received_at;
                working_chassis.valid_mask |= 1u << index;
            }
        }
        TickType_t now = xTaskGetTickCount();
        for (unsigned i = 0; i < DJI_FEEDBACK_MOTORS; ++i) {
            TickType_t age = now - last_rx[i];
            debug_motor[i].online = received[i] && age < MOTOR_TIMEOUT;
            debug_motor[i].age_ms = received[i] ?
                (uint32_t)((uint64_t)age * 1000u / configTICK_RATE_HZ) : UINT32_MAX;
        }
        taskENTER_CRITICAL();
        published_chassis = working_chassis;
        taskEXIT_CRITICAL();
        debug_motor_task_cycles++;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
    }
}

bool motor_feedback_init(void)
{
    if (!bsp_motor_can_init()) return false;
    /* FreeRTOS 栈深度单位是 StackType_t，F407 上 512 表示 2048 字节。 */
    return xTaskCreate(MotorFeedbackTask, "motor_rx", 512, NULL, 3, NULL) == pdPASS;
}
