#include "bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
uint32_t bsp_time_cycles(void)
{
    /* 只使能，不清零；多个任务可各自保存上次计数，互不影响。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return DWT->CYCCNT;
}
float bsp_time_cycles_to_seconds(uint32_t cycles)
{
    return (float)cycles / (float)SystemCoreClock;
}
/* 当前工程 tick=1kHz。上层以 uint32_t 无符号减法比较短时间差，可跨毫秒回绕。 */
uint32_t bsp_time_ms(void)
{
    return (uint32_t)((uint64_t)xTaskGetTickCount() * 1000u / configTICK_RATE_HZ);
}
uint32_t bsp_time_ms_from_isr(void)
{
    return (uint32_t)((uint64_t)xTaskGetTickCountFromISR() * 1000u / configTICK_RATE_HZ);
}
