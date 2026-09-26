#ifndef ATTITUDE_H
#define ATTITUDE_H
#include <stdbool.h>
typedef struct {
    unsigned stationary_samples;
    float gyro_sum[3], accel_sum[3], bias[3];
    bool initialized;
} AttitudeEstimator;
typedef struct {float yaw_deg,pitch_deg,yaw_rate_rad_s,pitch_rate_rad_s;bool valid;} AttitudeResult;
/* 当前沿用旧 EKF 的单实例状态：只允许 ins_task 调用。单位 rad/s、m/s²、秒。 */
void attitude_reset(AttitudeEstimator *state);
bool attitude_update(AttitudeEstimator *state,const float gyro[3],const float accel[3],float dt,AttitudeResult *out);
#endif
