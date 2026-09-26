#include "bsp_can.h"
#include "can.h"
#include "queue.h"
#include <assert.h>
#include <string.h>
#include <stdio.h>
static CAN_Registers regs, regs2;
CAN_HandleTypeDef hcan1 = {&regs}, hcan2 = {&regs2};
static BspCanTx tx_slot;
static CanTxDebug debug_chassis_tx;
static unsigned pending, defer_abort, fail_send, tx_count;
static unsigned multi_box, occupied[2], abort_count;
static uint16_t mailbox_id[2][3];
static unsigned bus_index(CAN_HandleTypeDef *can) { assert(can==&hcan1 || can==&hcan2); return can==&hcan2; }
static uint8_t last_tx[8];
static BspCanFrame queue_data[32];
static unsigned queue_len, start_count, stop_count;
static int fail_stage;
static CAN_FilterTypeDef last_filter;
static CAN_RxHeaderTypeDef rx_header = {0, 0, 8, 0x201};
static unsigned fifo_count;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *);
QueueHandle_t xQueueCreate(unsigned length, unsigned size) {
    assert(length == 32 && size == sizeof(BspCanFrame));
    queue_len = 0; return queue_data;
}
BaseType_t xQueueReceive(QueueHandle_t queue, void *out, TickType_t wait) {
    assert(queue == queue_data && wait == 0);
    if (!queue_len) return 0;
    memcpy(out, queue_data, sizeof(BspCanFrame));
    --queue_len;
    memmove(queue_data, queue_data + 1, queue_len * sizeof(BspCanFrame));
    return pdPASS;
}
BaseType_t xQueueSendFromISR(QueueHandle_t queue, const void *in, BaseType_t *wake) {
    assert(queue == queue_data); (void)wake;
    if (queue_len == 32) return 0;
    memcpy(&queue_data[queue_len++], in, sizeof(BspCanFrame));
    return pdPASS;
}
uint32_t bsp_time_ms_from_isr(void) { return 123; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can, CAN_FilterTypeDef *f) {
    assert((can == &hcan1 || can == &hcan2)); last_filter = *f; return fail_stage == 1 ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *can) {
    assert((can == &hcan1 || can == &hcan2)); start_count++; return fail_stage == 2 ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *can) {
    assert((can == &hcan1 || can == &hcan2)); stop_count++; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t flags) {
    assert((can == &hcan1 || can == &hcan2) && (flags & CAN_IT_RX_FIFO0_MSG_PENDING));
    return fail_stage == 3 ? HAL_ERROR : HAL_OK;
}
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *can) { assert((can == &hcan1 || can == &hcan2)); return 0x42; }
uint32_t HAL_CAN_GetState(CAN_HandleTypeDef *can) { assert((can == &hcan1 || can == &hcan2)); return HAL_CAN_STATE_LISTENING; }
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *can, uint32_t box) { if(multi_box) return occupied[bus_index(can)] & box; assert((can == &hcan1 || can == &hcan2) && box == 1); return pending; }
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can) { if(multi_box) { unsigned n=occupied[bus_index(can)]; return 3-((n&1)!=0)-((n&2)!=0)-((n&4)!=0); } assert((can == &hcan1 || can == &hcan2)); return 3-pending; }
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *can, uint32_t box) {
    if(multi_box) { abort_count++; if(!defer_abort) occupied[bus_index(can)] &= ~box; return HAL_OK; } assert((can == &hcan1 || can == &hcan2) && box == 1); if (!defer_abort) pending=0; return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can, CAN_TxHeaderTypeDef *header, uint8_t *bytes, uint32_t *box) {
    if(multi_box) {
        unsigned bus=bus_index(can), i=0;
        if(fail_send) return HAL_ERROR;
        while(i<3 && (occupied[bus] & (1u<<i))) i++;
        assert(i<3); *box=1u<<i; occupied[bus] |= *box;
        mailbox_id[bus][i]=header->StdId; memcpy(last_tx,bytes,8); tx_count++;
        return HAL_OK;
    }
    assert((can == &hcan1 || can == &hcan2) && !pending);
    assert(header->StdId == 0x200 && header->DLC == 8 && header->IDE == CAN_ID_STD && header->RTR == CAN_RTR_DATA);
    if (fail_send) return HAL_ERROR;
    memcpy(last_tx,bytes,8); *box=1; pending=1; tx_count++; return HAL_OK;
}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *can, uint32_t fifo) {
    assert((can == &hcan1 || can == &hcan2) && fifo == CAN_RX_FIFO0); return fifo_count;
}
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *can, uint32_t fifo, CAN_RxHeaderTypeDef *header, uint8_t *data) {
    assert((can == &hcan1 || can == &hcan2) && fifo == 0 && fifo_count);
    --fifo_count; *header = rx_header; memset(data, 0x12, 8); return HAL_OK;
}
int main(void)
{
    assert(!bsp_can_start(1)); /* No queue yet. */
    assert(bsp_can_init());
    for (int stage = 1; stage <= 3; ++stage) {
        fail_stage = stage;
        assert(!bsp_can_start(1));
        assert(debug_motor_can.started[0] == 0);
    }
    assert(debug_motor_can.start_failures[0] == 3 && stop_count == 1);
    fail_stage = 0;
    assert(bsp_can_start(1));
    assert(last_filter.FilterBank == 0 && last_filter.SlaveStartFilterBank == 14);
    assert(last_filter.FilterMaskIdLow == 6);
    unsigned previous_starts = start_count;
    assert(bsp_can_start(1) && start_count == previous_starts);
    assert(debug_motor_can.started[1] == 0);
    CAN_HandleTypeDef other;
    fifo_count = 1;
    HAL_CAN_RxFifo0MsgPendingCallback(&other);
    assert(fifo_count == 1 && queue_len == 0);
    fifo_count = 5;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(fifo_count == 2 && queue_len == 3); /* Bounded ISR. */
    BspCanFrame packet;
    assert(bsp_can_receive(&packet));
    assert(packet.bus == 1 && packet.id == 0x201 && packet.received_at_ms == 123);
    assert(packet.length == 8 && packet.data[0] == 0x12);
    while (bsp_can_receive(&packet)) {}
    rx_header.DLC = 9; fifo_count = 1;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(!bsp_can_receive(&packet) && debug_motor_can.rejected[0] == 1);
    rx_header.DLC = 8;
    for (int i = 0; i < 33; ++i) { fifo_count = 1; HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); }
    assert(queue_len == 32 && debug_motor_can.queue_drops[0] == 1);
    HAL_CAN_ErrorCallback(&other);
    assert(debug_motor_can.errors[0] == 0);
    HAL_CAN_ErrorCallback(&hcan1);
    assert(debug_motor_can.errors[0] == 1 && debug_motor_can.last_error[0] == 0x42);

    /* 任意合法 DLC 都应交给上层，不在 CAN ISR 中假定一定是电机帧。 */
    while (bsp_can_receive(&packet)) {}
    rx_header.DLC = 3; fifo_count = 1;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    assert(bsp_can_receive(&packet) && packet.length == 3);
    BspCanFrame drive = {.bus=1,.id=0x200,.length=8,.data={0,100,0xff,0x9c,7,0xd0,0xf8,0x30}};
    BspCanFrame stop = {.bus=1,.id=0x200,.length=8};
    assert(bsp_can_send_latest(&tx_slot,&drive,&debug_chassis_tx));
    assert(tx_count == 1 && last_tx[0] == 0 && last_tx[1] == 100);
    defer_abort = 1;
    assert(!bsp_can_send_latest(&tx_slot,&stop,&debug_chassis_tx)); /* No busy-wait or queued stale stop. */
    assert(tx_count == 1 && debug_chassis_tx.busy == 1);
    defer_abort = 0;
    assert(bsp_can_send_latest(&tx_slot,&stop,&debug_chassis_tx));
    assert(tx_count == 2 && debug_chassis_tx.cancel_requests == 2);
    for (unsigned i=0;i<8;++i) assert(last_tx[i] == 0);
    regs.ESR = CAN_ESR_BOFF;
    assert(!bsp_can_send_latest(&tx_slot,&drive,&debug_chassis_tx) && !pending); /* Cancel stale packet even on bus-off. */
    regs.ESR = 0; fail_send = 1;
    assert(!bsp_can_send_latest(&tx_slot,&drive,&debug_chassis_tx) && debug_chassis_tx.errors == 1);
    assert(!bsp_can_send_latest(&tx_slot,NULL,&debug_chassis_tx));

    BspCanFrame unrelated = {.bus=1,.id=0x1ff,.length=8};
    assert(!bsp_can_send_latest(&tx_slot,&unrelated,&debug_chassis_tx));

    /* CAN2 有自己的过滤器和计数；两条总线共用一个接收队列但保留 bus 标签。 */
    assert(!bsp_can_start(0) && !bsp_can_start(3));
    fail_stage=2;
    assert(!bsp_can_start(2) && bsp_can_is_ready(1));
    assert(debug_motor_can.started[1]==0 && debug_motor_can.start_failures[1]==1);
    fail_stage=0;fail_send=0;
    assert(bsp_can_start(2) && bsp_can_is_ready(2));
    assert(last_filter.FilterBank==14 && last_filter.SlaveStartFilterBank==14);
    while(bsp_can_receive(&packet)) {}
    rx_header.StdId=0x208;rx_header.DLC=8;fifo_count=1;
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
    assert(bsp_can_receive(&packet) && packet.bus==2 && packet.id==0x208);
    assert(debug_motor_can.rx_frames[1]==1 && debug_motor_can.rx_frames[0]>1);
    HAL_CAN_ErrorCallback(&hcan2);
    assert(debug_motor_can.errors[1]==1 && debug_motor_can.errors[0]==1);
    regs2.ESR=CAN_ESR_BOFF;
    assert(!bsp_can_is_ready(2) && bsp_can_is_ready(1));
    regs2.ESR=0;
    BspCanTx second_slot={0};drive.bus=2;
    assert(bsp_can_send_latest(&second_slot,&drive,&debug_chassis_tx));
    /* Regression: hardware pending belongs to the NEW mailbox occupant. */
    multi_box=1;
    BspCanTx pitch={0}, friction={0}, third={0}, other_bus={0};
    CanTxDebug pitch_debug={0}, friction_debug={0}, third_debug={0};
    BspCanFrame p={.bus=2,.id=0x1ff,.length=8}, fdrive={.bus=2,.id=0x200,.length=8,.data={1,2}},
        fstop={.bus=2,.id=0x200,.length=8}, extra={.bus=2,.id=0x2ff,.length=8};
    assert(bsp_can_send_latest(&pitch,&p,&pitch_debug));
    occupied[1]=0; /* pitch completed; mailbox 0 is free */
    assert(bsp_can_send_latest(&friction,&fdrive,&friction_debug)); /* reuses 0 */
    unsigned cancelled=abort_count;
    assert(bsp_can_send_latest(&pitch,&p,&pitch_debug)); /* must use 1 */
    assert(abort_count==cancelled); /* must never abort friction */
    assert(occupied[1]==3 && mailbox_id[1][0]==0x200 && mailbox_id[1][1]==0x1ff);
    assert(bsp_can_send_latest(&third,&extra,&third_debug)); /* mailbox 2 */
    assert(occupied[1]==7);
    defer_abort=1;
    assert(!bsp_can_send_latest(&friction,&fstop,&friction_debug));
    assert(occupied[1]==7); /* asynchronous abort: no busy-wait */
    defer_abort=0;
    assert(bsp_can_send_latest(&friction,&fstop,&friction_debug));
    assert(occupied[1]==7 && mailbox_id[1][1]==0x1ff && mailbox_id[1][2]==0x2ff);
    for(unsigned i=0;i<8;i++) assert(last_tx[i]==0);
    p.bus=1;
    assert(bsp_can_send_latest(&other_bus,&p,&pitch_debug)); /* bus1 slot0 independent */
    cancelled=abort_count;
    occupied[1]=0;
    assert(bsp_can_send_latest(&friction,&fstop,&friction_debug));
    assert(abort_count==cancelled && occupied[0]==1);
    assert(bsp_can_send_latest(&other_bus,&p,&pitch_debug));
    assert(abort_count==cancelled+1 && occupied[1]==1);
    assert(bsp_can_send_latest(&third,&extra,&third_debug)); /* now slot1 */
    cancelled=abort_count;
    p.bus=2;
    assert(bsp_can_send_latest(&pitch,&p,&pitch_debug)); /* stale slot1 belongs to third */
    assert(abort_count==cancelled && mailbox_id[1][1]==0x2ff && mailbox_id[1][2]==0x1ff);
    occupied[1] &= ~4u; /* pitch completes */
    assert(bsp_can_send_latest(&third,&extra,&third_debug));
    occupied[1] &= ~2u;
    assert(bsp_can_send_latest(&friction,&fdrive,&friction_debug));
    fail_send=1;
    assert(!bsp_can_send_latest(&pitch,&p,&pitch_debug));
    fail_send=0;
    assert(bsp_can_send_latest(&pitch,&p,&pitch_debug));
    puts("PASS: mailbox ownership/reuse, dual buses, three slots, stop replacement, deferred abort, TX failure, bounded RX");
    return 0;
}
