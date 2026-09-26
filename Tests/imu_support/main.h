#pragma once
#include <stdint.h>
#define HAL_SPI_MODULE_ENABLED
typedef enum {HAL_OK,HAL_ERROR} HAL_StatusTypeDef;
typedef enum {GPIO_PIN_RESET,GPIO_PIN_SET} GPIO_PinState;
typedef int GPIO_TypeDef;
extern GPIO_TypeDef port_a,port_b;
#define GPIOA (&port_a)
#define GPIOB (&port_b)
#define GPIO_PIN_4 16
#define GPIO_PIN_0 1
void HAL_GPIO_WritePin(GPIO_TypeDef *,uint16_t,GPIO_PinState);

#define BMI088_ACC_CS_GPIO_Port GPIOA
#define BMI088_ACC_CS_Pin GPIO_PIN_4
#define BMI088_GYRO_CS_GPIO_Port GPIOB
#define BMI088_GYRO_CS_Pin GPIO_PIN_0
