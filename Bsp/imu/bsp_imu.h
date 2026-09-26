#ifndef BSP_IMU_H
#define BSP_IMU_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef enum { BMI088_ACCEL, BMI088_GYRO } Bmi088Chip;
/* 单一调用者为 ins_task；不在中断调用，不与其他任务抢 SPI1。 */
bool bsp_imu_ready(void);
bool bsp_imu_read(Bmi088Chip chip, uint8_t reg, uint8_t *data, size_t length);
bool bsp_imu_write(Bmi088Chip chip, uint8_t reg, uint8_t value);
void bsp_imu_delay_ms(uint32_t milliseconds);
#endif
