#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "shoot_control.h"
typedef struct {
  uint32_t faults, command_age_ms, gimbal_ready, motors_ready, can_ready;
} ShootTaskDebug;
extern volatile ShootTaskDebug debug_shoot_task;
bool shoot_init(void);
bool shoot_set_command(const ShootCommand *command);
void shoot_task_entry(void *argument);
#endif
