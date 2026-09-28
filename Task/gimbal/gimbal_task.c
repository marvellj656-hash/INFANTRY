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
  uint32_t generation = 0, last_faults = 0, fault_cycles = 0;
  uint32_t previous_cycles = bsp_time_cycles();
  for (;;) {
    TickType_t now = xTaskGetTickCount(), elapsed = now - previous;
    previous = now;
    uint32_t cycles = bsp_time_cycles();
    float control_dt = bsp_time_cycles_to_seconds(cycles - previous_cycles);
    previous_cycles = cycles;
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
    if (control_dt <= 0 || control_dt > 0.020f)
      faults |= GIMBAL_BLOCK_TIMING;
    for (unsigned i = 0; i < 2; i++)
      permit = permit && f[i].online && f[i].transport_ready;
    permit = permit && (uint32_t)(bsp_time_ms() - command.received_at_ms) <
                           ROBOT_REMOTE_TIMEOUT_MS;
    if (generation != ins.generation) {
      generation = ins.generation;
      gimbal_control_reset(&controller);
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
                          control_dt, &result);
    if (!dji_motor_set_outputs(motors, result.raw, 2)) {
      faults |= GIMBAL_BLOCK_OUTPUT;
      for (unsigned i=0;i<2;i++) {
        pid_clear(&controller.angle[i]); pid_clear(&controller.speed[i]);
      }
      result = (GimbalOutput){.mode=controller.mode};
    }
    result.reference_generation = ins.generation;
    uint32_t runtime_faults = faults & ~(GIMBAL_BLOCK_COMMAND | GIMBAL_WAIT_NEUTRAL);
    if (runtime_faults) { last_faults = runtime_faults; ++fault_cycles; }
    float pitch_measured = result.mode == GIMBAL_INIT ?
        input.relative_deg[1] : input.angle_deg[1];
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
                      .pitch_relative_deg = input.relative_deg[1],
                      .yaw_target_deg = result.target_deg[0],
                      .pitch_target_deg = result.target_deg[1],
                      .pitch_measured_deg = pitch_measured,
                      .pitch_error_deg = result.target_deg[1] - pitch_measured,
                      .pitch_rate_target_rad_s = result.rate_target_rad_s[1],
                      .pitch_rate_measured_rad_s = input.rate_rad_s[1],
                      .pitch_speed_p = controller.speed[1].Pout,
                      .pitch_speed_i = controller.speed[1].Iout,
                      .pitch_speed_d = controller.speed[1].Dout,
                      .pitch_ecd = f[1].measure.ecd,
                      .pitch_target_limited = result.pitch_target_limited,
                      .pitch_output_limited = result.pitch_output_limited,
                      .last_faults = last_faults, .fault_cycles = fault_cycles};
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
