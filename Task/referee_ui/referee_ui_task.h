#ifndef REFEREE_UI_TASK_H
#define REFEREE_UI_TASK_H
#include <stdint.h>
typedef struct
{
    uint32_t queued, rejected, redraws;
    uint16_t client_id;
    uint8_t next_element, online;
} RefereeUiDebug;
extern volatile RefereeUiDebug debug_referee_ui;
/* 可从cmd任务请求重画；用计数锁存，100ms的UI任务不会漏掉10ms的按键事件。 */
void referee_ui_request_redraw(void);
void referee_ui_task_entry(void *argument);
#endif
