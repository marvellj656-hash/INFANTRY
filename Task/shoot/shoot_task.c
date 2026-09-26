#include "shoot_task.h"
#include "FreeRTOS.h"
#include "bsp_time.h"
#include "gimbal/gimbal_task.h"
#include "motor/dji_motor.h"
#include "queue.h"
#include "robot_config.h"
#include "shoot_config.h"
#include "task.h"
static QueueHandle_t commands;
static DjiMotorHandle motors[3];
static ShootController controller;
volatile ShootOutput debug_shoot;
volatile ShootTaskDebug debug_shoot_task;
bool shoot_init(void) {
  motors[0] = dji_motor_find(MOTOR_FRICTION_L);
  motors[1] = dji_motor_find(MOTOR_FRICTION_R);
  motors[2] = dji_motor_find(MOTOR_TRIGGER);
  commands = xQueueCreate(1, sizeof(ShootCommand));
  shoot_control_reset(&controller);
  return commands && motors[0] && motors[1] && motors[2];
}
bool shoot_set_command(const ShootCommand *command) {
  return commands && command && xQueueOverwrite(commands, command) == pdPASS;
}
void shoot_task_entry(void *argument) {
  (void)argument;
  TickType_t wake = xTaskGetTickCount(), previous = wake;
  ShootCommand command = {0};
  uint32_t generation = 0;
  for (;;) {
    TickType_t now = xTaskGetTickCount(), elapsed = now - previous;
    previous = now;
    (void)xQueueReceive(commands, &command, 0);
    DjiMotorFeedback f[3] = {0};
    GimbalOutput gimbal = {0};
    bool permit = dji_motor_read_many(motors, 3, f);
    bool motors_ready = permit, can_ready = permit;
    for (unsigned i = 0; i < 3; i++) {
      motors_ready &= f[i].online;
      can_ready &= f[i].transport_ready;
    }
    bool gimbal_ready = gimbal_get_state(&gimbal) && gimbal.centered;
    uint32_t age = bsp_time_ms() - command.received_at_ms;
    uint32_t faults =
        (!command.enabled || age >= ROBOT_REMOTE_TIMEOUT_MS) ? 1 : 0;
    if (!motors_ready)
      faults |= 2;
    if (!can_ready)
      faults |= 4;
    if (!elapsed || elapsed > pdMS_TO_TICKS(20))
      faults |= 8;
    if (!gimbal_ready)
      faults |= 16;
    if (!command.friction)
      faults |= 32;
    for (unsigned i = 0; i < 3; i++)
      permit = permit && f[i].online && f[i].transport_ready;
    permit = permit && gimbal_ready;
    permit = permit && (uint32_t)(bsp_time_ms() - command.received_at_ms) <
                           ROBOT_REMOTE_TIMEOUT_MS;
    if (generation != f[2].generation) {
      generation = f[2].generation;
      shoot_control_reset(&controller);
    }
    ShootInput input = {.rpm = {f[0].measure.speed_rpm, f[1].measure.speed_rpm,
                                f[2].measure.speed_rpm},
                        .trigger_angle_deg = f[2].total_angle_deg,
                        .trigger_current_raw = f[2].measure.current_raw,
                        .valid = permit};
    ShootOutput out;
    shoot_control_update(&controller, &command, &input, permit,
                         (float)elapsed / configTICK_RATE_HZ, &out);
    if (!dji_motor_set_outputs(motors, out.raw, 3)) {
      shoot_control_reset(&controller);
      out = (ShootOutput){0};
    }
    if (!out.armed && command.fire)
      faults |= 64;
    debug_shoot_task =
        (ShootTaskDebug){faults, age, gimbal_ready, motors_ready, can_ready};
    debug_shoot = out;
    if (elapsed > pdMS_TO_TICKS(20))
      wake = now;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SHOOT_PERIOD_MS));
  }
}
