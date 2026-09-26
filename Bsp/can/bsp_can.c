#include "bsp_can.h"
#include "bsp_time.h"
#include "can.h"     /* CubeMX 生成：这里只使用 extern hcan1/hcan2，不重新定义句柄。 */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#define RX_QUEUE_LENGTH 32u /* 最多暂存 32 帧；满了记录丢帧。 */
static QueueHandle_t rx_queue; /* ISR 放入，设备服务取走。 */
volatile CanDebug debug_motor_can;
static bool started[2];
/* 三个硬件邮箱像三个公共座位，不属于某个固定电机组。
 * 一帧发完，下一组就能坐到同一个座位。记录现在是谁占用，
 * 不能只拿上一圈记住的座位号去撤销，否则会撤掉别人的新指令。
 * 只有 motor_task 调用发送函数；ISR 不发送，也不修改这张表。 */
static const BspCanTx *mailbox_owner[2][3];
static unsigned mailbox_index(uint32_t mailbox)
{
    return mailbox == 1u ? 0u : mailbox == 2u ? 1u : 2u;
}
/* bus 是给上层看的 1/2，数组下标是 0/1；只有这里认识 HAL 句柄。 */
static CAN_HandleTypeDef *can_handle(uint8_t bus)
{
    if (bus == 1) return &hcan1;
    if (bus == 2) return &hcan2;
    return NULL;
}
static int can_index(CAN_HandleTypeDef *can)
{
    if (can == &hcan1) return 0;
    if (can == &hcan2) return 1;
    return -1;
}

/* 调用顺序：
 * main -> MX_CAN1_Init()/MX_CAN2_Init()：时钟、PD0/PD1、波特率、NVIC、HAL_CAN_Init。
 * robot_init -> 本函数：创建软件队列。
 * motor_task_entry -> bsp_can_start：设置过滤器并真正启动通信。
 * 注意：HAL_CAN_Init 只配置外设，不等于 HAL_CAN_Start。
 * 前者解决“CAN 硬件刚收到数据，CPU暂时还没来得及取”；后者解决“ISR已经把硬件数据取出来了，但任务暂时还没来得及处理”。这其实就是两级缓冲。
 */
bool bsp_can_init(void)
{
    /* 队列复制的是整个报文结构体，不是指向中断局部变量的指针（生命周期）。 */
    rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(BspCanFrame));
    return rx_queue != NULL;
}

bool bsp_can_start(uint8_t bus)
{
    CAN_HandleTypeDef *can = can_handle(bus);
    if (!rx_queue || !can) return false;
    unsigned index = bus - 1u;
    if (started[index]) return true;

    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = index == 0 ? 0 : 14;             /* CAN1 用第 0 组，CAN2 用第 14 组，都进各自 FIFO0。*/
    filter.SlaveStartFilterBank = 14;   /* 两个控制器共享过滤器组；分界必须一致：0..13 CAN1，14..27 CAN2。 */
    filter.FilterMode = CAN_FILTERMODE_IDMASK;//1 represent must compare filter,0 :don't care about
    filter.FilterScale = CAN_FILTERSCALE_32BIT;//width
    /* 掩码位=1 表示必须比较，=0 表示忽略。
     * 0x0006 比较 IDE、RTR 两位；目标值都是 0，即标准数据帧。
     * ID 位不比较，所以所有标准 ID 都可进入 FIFO；任务再按电机 ID 筛选。
     * 标准帧：11 位 ID；数据帧：携带数据，与远程请求帧相区分。
     */
    filter.FilterMaskIdLow = 0x0006;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    if (HAL_CAN_ConfigFilter(can, &filter) != HAL_OK) goto failed;

    /* 从初始化状态进入通信状态；本工程已由你在 CubeMX 配成 Normal。
     * 此函数内部可能等待硬件状态，所以从任务调用，不放进临界区/中断。
     */
    if (HAL_CAN_Start(can) != HAL_OK) goto failed;

    /* CubeMX 打开的是 NVIC；这里打开 CAN 外设内部的事件通知。
     * 两层都打开，接收事件才会进入 IRQ -> HAL -> 我们的回调。
     */





    /* 合并需要开启的 CAN 事件。 */
    uint32_t notifications = CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_ERROR | CAN_IT_BUSOFF |
        CAN_IT_ERROR_PASSIVE | CAN_IT_ERROR_WARNING | CAN_IT_LAST_ERROR_CODE;
/* 打开外设事件源；只打开 NVIC 还不够。 */
    if (HAL_CAN_ActivateNotification(can, notifications) != HAL_OK) {
        uint32_t error = HAL_CAN_GetError(can);
        (void)HAL_CAN_Stop(can);
        debug_motor_can.last_error[index] = error;
        debug_motor_can.start_failures[index]++;
        return false;
    }

    started[index] = true;
    debug_motor_can.started[index] = 1; /* 启动成功，不代表已收到电机反馈。 */
    return true;

failed:
    /* 共用失败出口；goto 只用于集中记录启动错误，不参与正常接收流程。 */
    debug_motor_can.start_failures[index]++;
    debug_motor_can.last_error[index] = HAL_CAN_GetError(can);
    return false;
}

/* 任务调用：有报文就复制到 *packet，没有就立即返回 false。 */
bool bsp_can_receive(BspCanFrame *packet)
{
    return rx_queue && packet && xQueueReceive(rx_queue, packet, 0) == pdPASS;
}

/* 不需要手动调用！CubeMX 的 CAN1_RX0_IRQHandler 调用 HAL_CAN_IRQHandler，
 * HAL 判断 FIFO0 有数据后，再调用这个同名回调。
 * 此时仍处于中断上下文，所以必须用 FreeRTOS 的 FromISR 接口。
 */
//IQR-->nvic->cpu->ISR HAL_CAN_IRQHandler()
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can)
{
    int index = can_index(can);
    if (index < 0 || !rx_queue) return;
    BaseType_t wake = pdFALSE;//初始未请求调度，队列操作可能将 wake 置为 pdTRUE

    /* bxCAN 的硬件 FIFO 最多存 3 帧。一次最多取 3 帧，限制中断执行时间。
     * 中途又到达的数据仍可触发后续中断，不在这里无限处理。
     */
    for (unsigned n = 0; n < 3 && HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0); ++n) {
        CAN_RxHeaderTypeDef header;//a frame of CAN include ID IDE(BASE OR EX) RTR REMOTE DLC :LEN
        BspCanFrame packet = {0};

        /* header 得到 ID/帧类型/长度；packet.data 得到最多 8 个数据字节。 */
        if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &header, packet.data) != HAL_OK) {//take away one frame
            debug_motor_can.errors[index]++;
            debug_motor_can.last_error[index] = HAL_CAN_GetError(can);
            break;//fail
        }
        debug_motor_can.rx_frames[index]++;
        /* BSP 只筛选标准数据帧；长度是否符合电机/超电协议交给相应模块判断。 */
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8) {//11bit ID  RTR==DATA  DLC==8
            debug_motor_can.rejected[index]++;
            continue;//skip
        }


        //SUMMERIZING:
        packet.bus = (uint8_t)(index + 1); /* 同一 CAN ID 在两条总线上属于不同设备。 */
        packet.id = (uint16_t)header.StdId;
        packet.length = (uint8_t)header.DLC;
        packet.received_at_ms = bsp_time_ms_from_isr();

        /* CAN 同时接收多个电机，不能像单遥控器那样只覆盖一份最新报文。
         * 队列保留多帧；满时记录丢包，任务里用时间戳判断反馈是否过期。
         */
        //TODO:using send ,avoid cover
        if (xQueueSendFromISR(rx_queue, &packet, &wake) != pdPASS)
            debug_motor_can.queue_drops[index]++;//producer>consumer
    }
    /* 如果队列操作唤醒了更高优先级任务，请求退出中断时调度它。 */
    portYIELD_FROM_ISR(wake); /* wake 表示是否唤醒了更高优先级任务；本工程 receive 不阻塞，通常为 false。 */
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *can)
{
    int index = can_index(can);
    if (index < 0) return;
    /* 只记录错误；不要在这里打印或等待硬件重新初始化。 */
    debug_motor_can.errors[index]++;
    debug_motor_can.last_error[index] = HAL_CAN_GetError(can);
}

/* 不再定义 CAN1_RX0_IRQHandler / CAN1_SCE_IRQHandler：
 * 唯一入口保留在 CubeMX 生成的 Core/Src/stm32f4xx_it.c 中。
 */

bool bsp_can_is_ready(uint8_t bus)
{
    CAN_HandleTypeDef *can = can_handle(bus);
    //&& 是短路求值
    return can && started[bus - 1u] && HAL_CAN_GetState(can) == HAL_CAN_STATE_LISTENING &&
        !(can->Instance->ESR & CAN_ESR_BOFF);
}
/* 固定负责某个 CAN ID；旧帧未发完时尝试取消，只保留最新控制指令。 */
/* slot：记录该 ID 的发送状态。 */
bool bsp_can_send_latest(BspCanTx *slot, const BspCanFrame *frame, volatile CanTxDebug *debug)
{
    if (!slot || !frame || !debug || frame->bus < 1 || frame->bus > 2 || frame->id > 0x7ff ||
        frame->length > 8 || !started[frame->bus - 1u]) return false;//写的这么严谨，貌似把所有可能出问题的都往上摆，那系统就一定不会出问题吗，真的要这么繁琐吗？？？？
    CAN_HandleTypeDef *can = can_handle(frame->bus);
    /* 防止错误复用发送槽，取消别的 ID 的报文。 */
    if (slot->bound && (slot->bus != frame->bus || slot->id != frame->id))
        return false;
    slot->bound = true;
    slot->bus = frame->bus;
    slot->id = frame->id;
    //cancel old frame
    bool owns_mailbox = slot->pending &&
        mailbox_owner[frame->bus - 1u][mailbox_index(slot->mailbox)] == slot;
    if (owns_mailbox && HAL_CAN_IsTxMessagePending(can, slot->mailbox)) {
        debug->cancel_requests++;
        if (HAL_CAN_AbortTxRequest(can, slot->mailbox) != HAL_OK) {
            debug->errors++; return false;
        }
        /* 硬件取消是异步的；未完成则返回，下周期重试，禁止在任务里忙等。 */
        if (HAL_CAN_IsTxMessagePending(can, slot->mailbox)) {
            debug->busy++; return false;
        }
    }
    slot->pending = false;
    if (!bsp_can_is_ready(frame->bus))
        return false;
    if (!HAL_CAN_GetTxMailboxesFreeLevel(can))
        { debug->busy++; return false; }
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = frame->id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = frame->length;
    header.TransmitGlobalTime = DISABLE;
    uint8_t data[8];
    for (unsigned i = 0; i < 8; ++i) data[i] = frame->data[i];
    if (HAL_CAN_AddTxMessage(can, &header, data, &slot->mailbox) != HAL_OK) {
        debug->errors++; return false;
    }
    /* HAL 刚告诉我们用了哪个邮箱，此时才更新归属。提交失败不能换主人。
     * 旧主人下次再来时，会发现座位已换人，不会误撤销本帧。 */
    mailbox_owner[frame->bus - 1u][mailbox_index(slot->mailbox)] = slot;
    slot->pending = true;
    debug->queued++;
    return true;
}
