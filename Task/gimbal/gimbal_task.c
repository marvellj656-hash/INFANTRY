#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "bsp_time.h"
#include "gimbal_config.h"
#include "ins/ins_task.h"
#include "motor/dji_motor.h"
#include "queue.h"
#include "robot_config.h"
#include "task.h"
volatile GimbalDebug debug_gimbal;
static QueueHandle_t commands;
static DjiMotorHandle motors[2];
static GimbalController controller;
static GimbalOutput state;
static uint32_t published_at;
static bool published;
bool gimbal_init(void) {
  motors[0] = dji_motor_find(MOTOR_GIMBAL_YAW);
  motors[1] = dji_motor_find(MOTOR_GIMBAL_PITCH);
  commands = xQueueCreate(1, sizeof(GimbalCommand));
  gimbal_control_reset(&controller);
  return commands && motors[0] && motors[1];
}
bool gimbal_set_command(const GimbalCommand *command) {
  return commands && command && xQueueOverwrite(commands, command) == pdPASS;
}
bool gimbal_get_state(GimbalOutput *out) {
  if (!out)
    return false;
  taskENTER_CRITICAL();
  *out = state;
  uint32_t stamp = published_at;
  bool valid = published;
  taskEXIT_CRITICAL();
  return valid && (uint32_t)(bsp_time_ms() - stamp) < 20;
}
void gimbal_task_entry(void *argument) {
  (void)argument;
  TickType_t wake = xTaskGetTickCount(), previous = wake;
  GimbalCommand command = {0};
  uint32_t generation = 0;
  for (;;) {
    TickType_t now = xTaskGetTickCount(), elapsed = now - previous;
    previous = now;
    (void)xQueueReceive(commands, &command, 0);
    DjiMotorFeedback f[2] = {0};
    InsState ins = {0};
    bool found = dji_motor_read_many(motors, 2, f);
    bool imu_ok = ins_get_state(&ins);
    bool permit = found && imu_ok;
    uint32_t age = bsp_time_ms() - command.received_at_ms;
    uint32_t faults = 0;
    if (!command.enabled || age >= ROBOT_REMOTE_TIMEOUT_MS)
      faults |= GIMBAL_BLOCK_COMMAND;
    if (!found || !f[0].online || !f[1].online)
      faults |= GIMBAL_BLOCK_MOTOR;
    if (!f[0].transport_ready || !f[1].transport_ready)
      faults |= GIMBAL_BLOCK_CAN;
    if (!imu_ok)
      faults |= GIMBAL_BLOCK_IMU;
    if (!elapsed || elapsed > pdMS_TO_TICKS(20))
      faults |= GIMBAL_BLOCK_TIMING;
    for (unsigned i = 0; i < 2; i++)
      permit = permit && f[i].online && f[i].transport_ready;
    permit = permit && (uint32_t)(bsp_time_ms() - command.received_at_ms) <
                           ROBOT_REMOTE_TIMEOUT_MS;
    if (generation != ins.generation) {
      generation = ins.generation;
      bool fault_latched = controller.fault_latched;
      gimbal_control_reset(&controller);
      controller.fault_latched = fault_latched;
    }
    GimbalInput input = {
        .relative_deg = {gimbal_encoder_relative(f[0].measure.ecd,
                                                 CENTER_ECD_YAW, -1),
                         gimbal_encoder_relative(f[1].measure.ecd,
                                                 CENTER_ECD_PITCH, 1)},
        .angle_deg = {ins.yaw_deg, ins.pitch_deg},
        .rate_rad_s = {ins.yaw_rate_rad_s, ins.pitch_rate_rad_s},
        .valid = permit};
    GimbalOutput result;
    gimbal_control_update(&controller, &command, &input, permit,
                          (float)elapsed / configTICK_RATE_HZ, &result);
    if (!dji_motor_set_outputs(motors, result.raw, 2)) {
      gimbal_control_reset(&controller);
      result = (GimbalOutput){0};
    }
    result.reference_generation = ins.generation;
    if (result.mode == GIMBAL_RELAX && command.enabled && !command.neutral)
      faults |= GIMBAL_WAIT_NEUTRAL;
    if (result.mode == GIMBAL_FAULT)
      faults |= GIMBAL_HOME_FAILED;
    debug_gimbal =
        (GimbalDebug){.faults = faults,
                      .mode = result.mode,
                      .imu_valid = imu_ok,
                      .neutral = command.neutral,
                      .yaw_online = f[0].online,
                      .pitch_online = f[1].online,
                      .tx_ready = f[0].transport_ready && f[1].transport_ready,
                      .command_age_ms = age,
                      .yaw_output = result.raw[0],
                      .pitch_output = result.raw[1],
                      .yaw_relative_deg = input.relative_deg[0],
                      .pitch_relative_deg = input.relative_deg[1]};
    taskENTER_CRITICAL();
    state = result;
    published_at = bsp_time_ms();
    published = true;
    taskEXIT_CRITICAL();
    if (elapsed > pdMS_TO_TICKS(20))
      wake = now;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(GIMBAL_PERIOD));
  }
}
