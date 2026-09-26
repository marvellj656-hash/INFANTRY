#pragma once
#include "main.h"
typedef struct {void *Instance;} SPI_HandleTypeDef;
extern SPI_HandleTypeDef hspi1;
#define SPI1 ((void*)1)
#define HAL_SPI_STATE_READY 1
unsigned HAL_SPI_GetState(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *,uint8_t *,uint8_t *,uint16_t,uint32_t);
