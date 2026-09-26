#include "bsp_imu.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#ifdef HAL_SPI_MODULE_ENABLED
#include "spi.h"
bool bsp_imu_ready(void)
{
    return hspi1.Instance == SPI1 && HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_READY;
}
static void select_chip(Bmi088Chip chip, GPIO_PinState level)
{
    /* 使用 CubeMX 生成的名字；以后改引脚，只改 .ioc 并重新生成即可。 */
    HAL_GPIO_WritePin(chip == BMI088_ACCEL ? BMI088_ACC_CS_GPIO_Port : BMI088_GYRO_CS_GPIO_Port,
                      chip == BMI088_ACCEL ? BMI088_ACC_CS_Pin : BMI088_GYRO_CS_Pin, level);
}
bool bsp_imu_read(Bmi088Chip chip, uint8_t reg, uint8_t *data, size_t length)
{
    if (!data || length == 0 || length > 16 || !bsp_imu_ready()) return false;
    uint8_t tx[18] = {0}, rx[18] = {0};
    /* bit7=1 为读。ACC 比 GYRO 多一个 dummy byte，这个字节不能当数据。 */
    unsigned skip = chip == BMI088_ACCEL ? 2u : 1u;
    tx[0] = reg | 0x80u;//1000 0000
    select_chip(chip, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(length+skip), 2);
    select_chip(chip, GPIO_PIN_SET); /* 即使传输失败也释放片选。 */
    if (status != HAL_OK) return false;
    memcpy(data, rx+skip, length);
    return true;
}
bool bsp_imu_write(Bmi088Chip chip, uint8_t reg, uint8_t value)
{
    if (!bsp_imu_ready()) return false;
    uint8_t tx[2] = {reg & 0x7fu, value}, rx[2];//0111 1111
    select_chip(chip, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 2);
    select_chip(chip, GPIO_PIN_SET);
    return status == HAL_OK;
}
#else
/* CubeMX 尚未配置 SPI1 时仍可编译底盘；不能伪造 IMU 就绪。 */
bool bsp_imu_ready(void) { return false; }
bool bsp_imu_read(Bmi088Chip chip,uint8_t reg,uint8_t *data,size_t length)
{ (void)chip;(void)reg;(void)data;(void)length;return false; }
bool bsp_imu_write(Bmi088Chip chip,uint8_t reg,uint8_t value)
{ (void)chip;(void)reg;(void)value;return false; }
#endif
void bsp_imu_delay_ms(uint32_t milliseconds)
{
    /* 初始化等待放在任务中，阻塞当前任务，不占用 CPU 忙等。 */
    TickType_t ticks = pdMS_TO_TICKS(milliseconds);
    vTaskDelay(ticks ? ticks : 1);
}
