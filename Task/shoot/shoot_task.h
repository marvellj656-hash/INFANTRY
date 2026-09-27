#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H
#include "shoot_control.h"

/* faults 可以同时出现多项，用按位或组合。例如 32+2=34。
 * WAIT 类表示正常等待条件，不一定是硬件损坏。 */
typedef enum {
  SHOOT_FAULT_DISABLED_OR_STALE = 1u,
  SHOOT_FAULT_MOTOR_OFFLINE = 2u,
  SHOOT_FAULT_CAN_NOT_READY = 4u,
  SHOOT_FAULT_TASK_TIMING = 8u,
  SHOOT_FAULT_GIMBAL_NOT_READY = 16u,
  SHOOT_FAULT_FRICTION_OFF = 32u,
  SHOOT_FAULT_WAIT_FIRE_RELEASE = 64u,
  SHOOT_FAULT_WAIT_FRICTION_SPEED = 128u,
  SHOOT_FAULT_JAM_LATCHED = 256u,
  SHOOT_FAULT_OUTPUT_REJECTED = 512u
} ShootFault;
typedef struct {
  uint32_t faults, command_age_ms, gimbal_ready, motors_ready, can_ready;
  uint32_t offline_mask; /* 位0=左摩擦、位1=右摩擦、位2=拨弹；可相加。 */
  uint32_t brake_timeouts; /* 制动超时次数：用于调参，不把一次瞬间事件漏掉。 */
} ShootTaskDebug;
extern volatile ShootTaskDebug debug_shoot_task;
bool shoot_init(void);
bool shoot_set_command(const ShootCommand *command);
void shoot_task_entry(void *argument);
#endif
