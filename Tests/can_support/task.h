#include "FreeRTOS.h"
TickType_t xTaskGetTickCountFromISR(void);
#define portYIELD_FROM_ISR(wake) ((void)(wake))

#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
