#include "FreeRTOS.h"
#include <stddef.h>
typedef void *QueueHandle_t;
QueueHandle_t xQueueCreate(unsigned length, unsigned bytes);
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait);
BaseType_t xQueueOverwriteFromISR(QueueHandle_t queue, const void *data, BaseType_t *wake);
BaseType_t xQueueReset(QueueHandle_t queue);
