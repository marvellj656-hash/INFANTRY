#include "referee_task.h"
#include "referee/referee.h"
#include "referee/referee_protocol.h"
#include "bsp_referee.h"
#include "bsp_time.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include <string.h>

#define REFEREE_TASK_PERIOD_MS 2u
#define REFEREE_TX_INTERVAL_MS 100u
#define REFEREE_TX_QUEUE_LENGTH 4u
#define INTERACTION_CAPACITY (REFEREE_PAYLOAD_CAPACITY - 6u)
typedef struct {
    uint16_t data_command;
    uint16_t receiver_id;
    uint16_t length;
    uint8_t sender_id;
    uint32_t generation;
    uint8_t data[INTERACTION_CAPACITY];
} InteractionRequest;

static QueueHandle_t tx_queue;
static StaticQueue_t tx_queue_object;
static uint8_t tx_queue_storage[REFEREE_TX_QUEUE_LENGTH * sizeof(InteractionRequest)];

bool referee_task_init(void)
{
    referee_init();
    tx_queue = xQueueCreateStatic(REFEREE_TX_QUEUE_LENGTH, sizeof(InteractionRequest),
                                 tx_queue_storage, &tx_queue_object);
    return tx_queue != NULL;
}

bool referee_send_interaction(uint16_t data_command, uint16_t receiver_id,
                              const uint8_t *data, size_t length)
{
    if (!tx_queue || receiver_id == 0u || length > INTERACTION_CAPACITY ||
        (!data && length != 0u)) return false;
    RefereeState state;
    if (!referee_get_state(&state) || !state.status.fresh || state.robot_id == 0u) return false;
    InteractionRequest request = {
        .data_command = data_command, .receiver_id = receiver_id, .length = (uint16_t)length,
        .sender_id = state.robot_id, .generation = state.generation
    };
    if (length) memcpy(request.data, data, length);
    /* 命令需要按顺序发送，因此使用 FIFO，不能用 overwrite 覆盖前一条。 */
    return xQueueSend(tx_queue, &request, 0) == pdPASS;
}

static bool request_is_current(const InteractionRequest *request)
{
    RefereeState state;
    return referee_get_state(&state) && state.status.fresh && state.robot_id != 0u &&
           state.robot_id == request->sender_id && state.generation == request->generation;
}

static bool send_interaction(const InteractionRequest *request, uint8_t sequence)
{
    uint8_t payload[REFEREE_PAYLOAD_CAPACITY];
    uint8_t frame[REFEREE_FRAME_CAPACITY];
    payload[0] = (uint8_t)request->data_command;
    payload[1] = (uint8_t)(request->data_command >> 8);
    payload[2] = request->sender_id;
    payload[3] = 0u;
    payload[4] = (uint8_t)request->receiver_id;
    payload[5] = (uint8_t)(request->receiver_id >> 8);
    if (request->length) memcpy(payload + 6, request->data, request->length);
    size_t length = referee_protocol_encode(0x0301u, sequence, payload,
                                            request->length + 6u, frame, sizeof(frame));
    /* BSP 必须复制到自己的发送缓冲区；不能在 DMA 中保留此栈上指针。 */
    return length != 0u && bsp_referee_send(frame, length);
}

void referee_task_entry(void *argument)
{
    (void)argument;
    uint32_t generation = bsp_referee_generation();
    uint32_t last_tx_ms = bsp_time_ms();
    uint8_t sequence = 0;
    InteractionRequest pending;
    bool has_pending = false;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        bool started = bsp_referee_start();
        uint32_t current_generation = bsp_referee_generation();
        if (current_generation != generation) {
            referee_reset_stream();
            generation = current_generation;
            if (tx_queue) (void)xQueueReset(tx_queue);
            has_pending = false;
        }
        if (started) {
            BspRefereePacket packet;
            /* 单一消费者。其他任务读快照，不允许来抢这条串口队列。 */
            for (unsigned n = 0; n < 16u && bsp_referee_receive(&packet); ++n) {
                /* 收包期间也可能发生 UART 错误/溢出，检查代次后再拼包。 */
                current_generation = bsp_referee_generation();
                if (current_generation != generation) {
                    referee_reset_stream();
                    generation = current_generation;
                    if (tx_queue) (void)xQueueReset(tx_queue);
                    has_pending = false;
                    /* 这包可能刚好取自故障前，不能再拿它刷新在线状态。 */
                    continue;
                }
                referee_receive(packet.data, packet.length, packet.received_at_ms);
            }
        }
        referee_update();
        RefereeState current;
        if (!referee_get_state(&current) || !current.status.fresh) {
            /* 掉线期间的旧画面/旧客户端地址不留到重新上线后补发。 */
            has_pending = false;
            if (tx_queue) (void)xQueueReset(tx_queue);
        }
        if (has_pending && !request_is_current(&pending)) has_pending = false;
        uint32_t now = bsp_time_ms();
        if (tx_queue && (uint32_t)(now - last_tx_ms) >= REFEREE_TX_INTERVAL_MS) {
            /* 只保留同一身份的请求；队列中的旧 sender/client 配对不能借新身份发送。 */
            for (unsigned n = 0; !has_pending && n < REFEREE_TX_QUEUE_LENGTH; ++n) {
                if (xQueueReceive(tx_queue, &pending, 0) != pdPASS) break;
                has_pending = request_is_current(&pending);
            }
            if (has_pending) {
                last_tx_ms = now;
                if (started && send_interaction(&pending, sequence)) {
                    has_pending = false;
                    ++sequence;
                    ++debug_referee.tx_accepted;
                } else ++debug_referee.tx_rejected;
                /* UART 只是暂忙：保持 pending，下次重试，避免 UI 的 ADD 永久丢失。
                 * 真故障会改变代次，下一圈先清掉旧请求并让 UI 重新建立整幅画面。 */
            }
        }
        ++debug_referee.task_cycles;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(REFEREE_TASK_PERIOD_MS));
    }
}
