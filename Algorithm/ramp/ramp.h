#ifndef ALGORITHM_RAMP_H
#define ALGORITHM_RAMP_H

/* 调用者保证加速度非负、dt_seconds 为正；XY 联合限加速度，避免斜向更猛。 */
void ramp_update_xy(float *x, float *y, float target_x, float target_y,
                    float acceleration, float dt_seconds);
float ramp_update(float current, float target, float acceleration, float dt_seconds);

#endif
