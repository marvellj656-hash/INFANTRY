#include "bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"
/* 当前工程 tick=1kHz。上层以 uint32_t 无符号减法比较短时间差，可跨毫秒回绕。 */
uint32_t bsp_time_ms(void)
{
    return (uint32_t)((uint64_t)xTaskGetTickCount() * 1000u / configTICK_RATE_HZ);
}
uint32_t bsp_time_ms_from_isr(void)
{
    return (uint32_t)((uint64_t)xTaskGetTickCountFromISR() * 1000u / configTICK_RATE_HZ);
}
