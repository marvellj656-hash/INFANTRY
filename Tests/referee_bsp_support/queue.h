#ifndef REFEREE_TEST_QUEUE_H
#define REFEREE_TEST_QUEUE_H
#include "FreeRTOS.h"
#include <stddef.h>
typedef struct {
    uint8_t *storage;
    unsigned length, bytes, count, head, tail;
} StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
QueueHandle_t xQueueCreateStatic(UBaseType_t length, UBaseType_t bytes,
                                 uint8_t *storage, StaticQueue_t *object);
QueueHandle_t xQueueCreate(unsigned length, unsigned bytes);
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait);
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *data, BaseType_t *wake);
BaseType_t xQueueOverwriteFromISR(QueueHandle_t queue, const void *data, BaseType_t *wake);
BaseType_t xQueueReset(QueueHandle_t queue);
#endif
