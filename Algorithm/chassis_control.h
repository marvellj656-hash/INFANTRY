#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { float vx, vy, wz; } ChassisVelocity; /* mm/s, mm/s, rad/s */
typedef struct {
    float integral, previous_error, previous_measure;
    bool initialized;
} ChassisPid;
typedef struct {
    ChassisVelocity velocity;
    ChassisPid pid[4];
    bool armed;
} ChassisController;
typedef struct {
    ChassisVelocity target, velocity;
    float target_rpm[4], measured_rpm[4];
    int16_t current_raw[4];
    float power_scale, estimated_power_w, budget_w;
    uint32_t armed, fault_flags;
} ChassisOutput;
enum {
    CHASSIS_FAULT_REMOTE = 1u,
    CHASSIS_FAULT_MOTOR = 2u,
    CHASSIS_FAULT_TIMING = 4u,
    CHASSIS_FAULT_CAN = 8u,
    CHASSIS_WAIT_NEUTRAL = 16u
};
void chassis_reset(ChassisController *controller);
ChassisVelocity chassis_target(int16_t ch1, int16_t ch2, int16_t ch3);
void chassis_inverse(ChassisVelocity velocity, float rpm[4]);
/* 纯算法：无 HAL、无任务/动态分配。permit=false 时立即零电流并清积分。
 * 首次使能或故障恢复后，要求本地遥控指令归零一次，不要求云台归中。
 */
void chassis_step(ChassisController *controller, ChassisVelocity target,
                  const float measured_rpm[4], bool permit, bool neutral,
                  float dt, float budget_w, ChassisOutput *out);
/* 统一缩放四路电流：保持电流比例，不向任何轮子反向增加电流。 */
float chassis_power_limit(float current_raw[4], const float measured_rpm[4],
                          float budget_w, float *estimated_w);
void chassis_pack_current(const int16_t current_raw[4], uint8_t bytes[8]);
#endif
