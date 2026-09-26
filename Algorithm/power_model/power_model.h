#ifndef ALGORITHM_POWER_MODEL_H
#define ALGORITHM_POWER_MODEL_H

typedef struct {
    float raw_to_amp, rotor_torque_constant, equivalent_resistance, idle_w;
} PowerModel;
/* 修改四轮电流、返回统一缩放系数。这里只计算，不决定比赛预算或超电模式。 */
float power_model_limit(const PowerModel *model, float current_raw[4],
                        const float measured_rpm[4], float budget_w, float *estimated_w);

#endif
