#include "bsp_motor_can.h"
#include "can.h"
#include "queue.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static CAN_Registers regs;
CAN_HandleTypeDef hcan1 = {&regs};
static unsigned pending, defer_abort, fail_send, tx_count;
static uint8_t last_tx[8];
static MotorCanPacket queue_data[32];
static unsigned queue_len, start_count, stop_count;
static int fail_stage;
static CAN_FilterTypeDef last_filter;
static CAN_RxHeaderTypeDef rx_header = {0, 0, 8, 0x201};
static unsigned fifo_count;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *);
QueueHandle_t xQueueCreate(unsigned length, unsigned size) {
    assert(length == 32 && size == sizeof(MotorCanPacket));
    queue_len = 0; return queue_data;
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait) {
    assert(queue == queue_data && wait == 0);
    if (!queue_len) return 0;
    memcpy(out, queue_data, sizeof(MotorCanPacket));
    --queue_len;
    memmove(queue_data, queue_data + 1, queue_len * sizeof(MotorCanPacket));
    return pdPASS;
}
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *in, BaseType_t *wake) {
    assert(queue == queue_data); (void)wake;
    if (queue_len == 32) return 0;
    memcpy(&queue_data[queue_len++], in, sizeof(MotorCanPacket));
    return pdPASS;
}
TickType_t xTaskGetTickCountFromISR(void) { return 123; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can, CAN_FilterTypeDef *f) {
    assert(can == &hcan1); last_filter = *f; return fail_stage == 1 ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *can) {
    assert(can == &hcan1); start_count++; return fail_stage == 2 ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *can) {
    assert(can == &hcan1); stop_count++; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t flags) {
    assert(can == &hcan1 && (flags & CAN_IT_RX_FIFO0_MSG_PENDING));
    return fail_stage == 3 ? HAL_ERROR : HAL_OK;
}
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *can) { assert(can == &hcan1); return 0x42; }
uint32_t HAL_CAN_GetState(CAN_HandleTypeDef *can) { assert(can == &hcan1); return HAL_CAN_STATE_LISTENING; }
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *can, uint32_t box) { assert(can == &hcan1 && box == 1); return pending; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can) { assert(can == &hcan1); return 3-pending; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *can, uint32_t box) {
    assert(can == &hcan1 && box == 1); if (!defer_abort) pending=0; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can, CAN_TxHeaderTypeDef *header, uint8_t *bytes, uint32_t *box) {
    assert(can == &hcan1 && !pending);
    assert(header->StdId == 0x200 && header->DLC == 8 && header->IDE == CAN_ID_STD && header->RTR == CAN_RTR_DATA);
    if (fail_send) return HAL_ERROR;
    memcpy(last_tx,bytes,8); *box=1; pending=1; tx_count++; return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *can, uint32_t fifo) {
    assert(can == &hcan1 && fifo == CAN_RX_FIFO0); return fifo_count;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *can, uint32_t fifo, CAN_RxHeaderTypeDef *header, uint8_t *data) {
    assert(can == &hcan1 && fifo == 0 && fifo_count);
    --fifo_count; *header = rx_header; memset(data, 0x12, 8); return HAL_OK;
}
int main(void)
{
    assert(!bsp_motor_can_start()); /* No queue yet. */
    assert(bsp_motor_can_init());
    for (int stage = 1; stage <= 3; ++stage) {
        fail_stage = stage;
        assert(!bsp_motor_can_start());
        assert(debug_motor_can.started[0] == 0);
    }
    assert(debug_motor_can.start_failures[0] == 3 && stop_count == 1);
    fail_stage = 0;
    assert(bsp_motor_can_start());
    assert(last_filter.FilterBank == 0 && last_filter.SlaveStartFilterBank == 14);
    assert(last_filter.FilterMaskIdLow == 6);
    unsigned previous_starts = start_count;
    assert(bsp_motor_can_start() && start_count == previous_starts);
    assert(debug_motor_can.started[1] == 0);
    CAN_HandleTypeDef other;
    fifo_count = 1;
    HAL_CAN_RxFifo0MsgPendingCallback(&other);
    assert(fifo_count == 1 && queue_len == 0);
    fifo_count = 5;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(fifo_count == 2 && queue_len == 3); /* Bounded ISR. */
    MotorCanPacket packet;
    assert(bsp_motor_can_take(&packet));
    assert(packet.bus == 1 && packet.id == 0x201 && packet.received_at == 123);
    assert(packet.length == 8 && packet.data[0] == 0x12);
    while (bsp_motor_can_take(&packet)) {}
    rx_header.DLC = 7; fifo_count = 1;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(!bsp_motor_can_take(&packet) && debug_motor_can.rejected[0] == 1);
    rx_header.DLC = 8;
    for (int i = 0; i < 33; ++i) { fifo_count = 1; HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); }
    assert(queue_len == 32 && debug_motor_can.queue_drops[0] == 1);
    HAL_CAN_ErrorCallback(&other);
    assert(debug_motor_can.errors[0] == 0);
    HAL_CAN_ErrorCallback(&hcan1);
    assert(debug_motor_can.errors[0] == 1 && debug_motor_can.last_error[0] == 0x42);
    int16_t drive[4] = {100,-100,2000,-2000}, stop[4] = {0};
    assert(bsp_motor_can_send_chassis(drive));
    assert(tx_count == 1 && last_tx[0] == 0 && last_tx[1] == 100);
    defer_abort = 1;
    assert(!bsp_motor_can_send_chassis(stop)); /* No busy-wait or queued stale stop. */
    assert(tx_count == 1 && debug_chassis_tx.busy == 1);
    defer_abort = 0;
    assert(bsp_motor_can_send_chassis(stop));
    assert(tx_count == 2 && debug_chassis_tx.cancel_requests == 2);
    for (unsigned i=0;i<8;++i) assert(last_tx[i] == 0);
    regs.ESR = CAN_ESR_BOFF;
    assert(!bsp_motor_can_send_chassis(drive) && !pending); /* Cancel stale packet even on bus-off. */
    regs.ESR = 0; fail_send = 1;
    assert(!bsp_motor_can_send_chassis(drive) && debug_chassis_tx.errors == 1);
    assert(!bsp_motor_can_send_chassis(NULL));
    puts("PASS: CAN RX, CubeMX handle, bounded ISR, queue drops, current frame, stop replaces pending drive, bus-off");
    return 0;
}
