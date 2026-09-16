#ifndef MOTOR_FEEDBACK_H
#define MOTOR_FEEDBACK_H
#include "dji_feedback.h"
#include "FreeRTOS.h"
typedef struct {
    uint32_t bus, id;
    uint16_t ecd;
    int16_t speed_rpm, current_raw;
    uint8_t temperature;
    uint32_t online, age_ms, frames, invalid_frames;
} MotorFeedbackDebug;
extern volatile MotorFeedbackDebug debug_motor[DJI_FEEDBACK_MOTORS];
extern volatile uint32_t debug_motor_task_cycles;
/* 控制任务只读完整反馈快照；调试结构不是控制数据源。 */
typedef struct {
    int16_t speed_rpm[4];
    TickType_t received_at[4];
    uint32_t valid_mask;
} ChassisFeedback;
void motor_feedback_get_chassis(ChassisFeedback *out);

bool motor_feedback_init(void);
#endif
