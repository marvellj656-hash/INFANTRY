#include "motor_task.h"
#include "motor/dji_motor_service.h"
#include "robot_config.h"
#include "bsp_can.h"
#include "bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"
volatile uint32_t debug_motor_task_cycles;
void motor_task_entry(void *argument)
{
    (void)argument;
    /* 独立启动，CAN2 故障不能阻止 CAN1 底盘接收。 */
    (void)bsp_can_start(1);
    (void)bsp_can_start(2);
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        BspCanFrame frame;
        /* 唯一接收者：取一帧后再按设备分发；不能让超电任务来抢同一接收队列。 */
        for (unsigned n = 0; n < 32 && bsp_can_receive(&frame); ++n) {
            if (dji_motor_receive(frame.bus, frame.id, frame.data, frame.length, frame.received_at_ms)) continue;
            /* 后续超级电容协议在此分发；尚未迁移时未识别帧不参与控制。 */
        }
        dji_motor_service();
        debug_motor_task_cycles++;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(ROBOT_MOTOR_PERIOD_MS));
    }
}
