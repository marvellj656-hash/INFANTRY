#include "FreeRTOS.h"
typedef void *QueueHandle_t;
QueueHandle_t xQueueCreate(unsigned length, unsigned size);
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait);
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *in, BaseType_t *wake);
