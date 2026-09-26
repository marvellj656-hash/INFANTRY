#include "ins_task.h"
#include "FreeRTOS.h"
#include "attitude.h"
#include "bsp_time.h"
#include "imu/bmi088.h"
#include "task.h"
#include <math.h>
volatile InsDebug debug_ins;
static Bmi088State bmi088;
static AttitudeEstimator estimator;
static InsState state;
bool ins_get_state(InsState *out) {
  if (!out)
    return false;
  taskENTER_CRITICAL();
  *out = state;
  taskEXIT_CRITICAL();
  out->valid =
      out->valid && (uint32_t)(bsp_time_ms() - out->received_at_ms) < 20;
  return out->valid;
}
void ins_task_entry(void *argument) {
  (void)argument;
  TickType_t wake = xTaskGetTickCount(), previous = wake;
  uint32_t generation = 0;
  bool was_initialized = false;
  for (;;) {
    if (!bmi088.initialized) {
      debug_ins.initialized = 0;
      debug_ins.valid = 0;
      attitude_reset(&estimator);
      taskENTER_CRITICAL();
      state.valid = false;
      taskEXIT_CRITICAL();
      if (!bmi088_init(&bmi088)) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        wake = previous = xTaskGetTickCount();
        continue;
      }
      wake = previous = xTaskGetTickCount();
    }
    bool sample = bmi088_read(&bmi088);
    debug_ins.initialized = bmi088.initialized;
    debug_ins.sample_errors = bmi088.errors;
    debug_ins.accel_norm = sqrtf(bmi088.accel[0] * bmi088.accel[0] +
                                 bmi088.accel[1] * bmi088.accel[1] +
                                 bmi088.accel[2] * bmi088.accel[2]);
    debug_ins.gyro_norm = sqrtf(bmi088.gyro[0] * bmi088.gyro[0] +
                                bmi088.gyro[1] * bmi088.gyro[1] +
                                bmi088.gyro[2] * bmi088.gyro[2]);
    TickType_t now = xTaskGetTickCount(), elapsed = now - previous;
    previous = now;
    AttitudeResult result = {0};
    if (sample)
      attitude_update(&estimator, bmi088.gyro, bmi088.accel,
                      (float)elapsed / configTICK_RATE_HZ, &result);
    else {
      debug_ins.initialized = 0;
      debug_ins.valid = 0;
      attitude_reset(&estimator);
    }
    if (estimator.initialized && !was_initialized)
      generation++;
    was_initialized = estimator.initialized;
    InsState next = {result.yaw_deg,        result.pitch_deg,
                     result.yaw_rate_rad_s, result.pitch_rate_rad_s,
                     bsp_time_ms(),         generation,
                     result.valid};
    taskENTER_CRITICAL();
    state = next;
    debug_ins.valid = next.valid;
    debug_ins.stationary_samples = estimator.stationary_samples;
    debug_ins.generation = generation;
    debug_ins.yaw_deg = next.yaw_deg;
    debug_ins.pitch_deg = next.pitch_deg;
    taskEXIT_CRITICAL();
    if (elapsed > pdMS_TO_TICKS(2))
      wake = now;
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(2));
  }
}
