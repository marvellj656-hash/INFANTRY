#ifndef BSP_BSP_TIME_H
#define BSP_BSP_TIME_H

#include <stdint.h>
/* 控制器使用与RTT相同的DWT周期计时，避免1ms Tick量化积分/微分时间。 */
uint32_t bsp_time_cycles(void);
float bsp_time_cycles_to_seconds(uint32_t cycles);
uint32_t bsp_time_ms(void);
uint32_t bsp_time_ms_from_isr(void);

#endif
