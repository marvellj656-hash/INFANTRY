#ifndef BSP_BSP_CAN_H
#define BSP_BSP_CAN_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint32_t received_at_ms; /* 硬件接收时记录，队列等待不能让旧帧变成新帧。 */
    uint16_t id;
    uint8_t bus, length, data[8];
} BspCanFrame;
typedef struct {
    uint32_t started[2], rx_frames[2], queue_drops[2], rejected[2];
    uint32_t errors[2], last_error[2], start_failures[2];
} CanDebug;
extern volatile CanDebug debug_motor_can;
typedef struct {
    uint32_t queued;          /* HAL 接受的新请求次数，不是电调执行次数。 */
    uint32_t cancel_requests; /* 请求撤销尚未发出的旧帧次数。 */
    uint32_t busy;            /* 邮箱未空、当前发送未被接受次数。 */
    uint32_t errors;          /* HAL 报错次数。 */
} CanTxDebug;
/* 每个 bus/id 一份软件发送槽，仅由电机服务访问。 */
typedef struct {
    uint32_t mailbox; /* HAL 的硬件邮箱位掩码。 */
    uint16_t id;
    uint8_t bus;
    bool pending;    /* 已提交的请求仍在等待完成。 */
    bool bound;      /* 该槽已经绑定固定的 bus/id。 */
} BspCanTx;
bool bsp_can_init(void);
bool bsp_can_start(uint8_t bus); /* 两路独立启动，失败互不阻断。 */
bool bsp_can_receive(BspCanFrame *frame); /* 唯一消费者 motor_task。 */
bool bsp_can_is_ready(uint8_t bus);
/* 非阻塞，尝试替换本槽旧请求。true 只表示 HAL 接受，不代表已经上总线。 */
bool bsp_can_send_latest(BspCanTx *slot,const BspCanFrame *frame,volatile CanTxDebug *debug);
#endif
