#ifndef MODULES_BMI088_H
#define MODULES_BMI088_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
  uint8_t accel_id, gyro_id;
  bool initialized, valid;
  float accel[3]; /* 传感器坐标 m/s^2；此处不做安装方向变换。 */
  float gyro[3];  /* 传感器坐标 rad/s；尚未扣除本机实测零偏。 */
  float temperature_c;
  uint32_t samples, errors;
} Bmi088State;
typedef struct {
  uint32_t init_attempts, init_failures, reg, actual, expected, mask, accel_id,
      gyro_id;
} Bmi088Debug;
extern volatile Bmi088Debug debug_bmi088;
bool bmi088_init(Bmi088State *imu);
bool bmi088_read(Bmi088State *imu);
#endif
