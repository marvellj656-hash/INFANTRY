#ifndef TEST_USART_H
#define TEST_USART_H
#include <stdint.h>
typedef struct { uint32_t ErrorCode; } UART_HandleTypeDef;
extern UART_HandleTypeDef huart3;
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
#define HAL_UART_RXEVENT_IDLE 1
#define HAL_UART_RXEVENT_HT 2
#define HAL_UART_RXEVENT_TC 3
#define __HAL_UART_CLEAR_OREFLAG(x) ((void)(x))
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *b, uint16_t n);
int HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *h);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h);
#endif
