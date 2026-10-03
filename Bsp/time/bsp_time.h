#ifndef BSP_BSP_TIME_H
#define BSP_BSP_TIME_H

#include <stdint.h>
uint32_t bsp_time_ms(void);
uint32_t bsp_time_ms_from_isr(void);

#endif
