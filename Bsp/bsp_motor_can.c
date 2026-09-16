#include "bsp_motor_can.h"
#include "chassis_control.h"
#include "can.h"     /* CubeMX 生成：这里只使用 extern hcan1，不重新定义句柄。 */
#include "queue.h"
#include "task.h"

#define RX_QUEUE_LENGTH 32u//个数，32 * motorcanpacket
static QueueHandle_t rx_queue;//very big
volatile MotorCanDebug debug_motor_can;

/* 调用顺序：
 * main -> MX_CAN1_Init()：时钟、PD0/PD1、波特率、NVIC、HAL_CAN_Init。
 * robot_app_init -> motor_feedback_init -> 本函数：创建软件队列。
 * MotorFeedbackTask -> bsp_motor_can_start：设置过滤器并真正启动通信。
 * 注意：HAL_CAN_Init 只配置外设，不等于 HAL_CAN_Start。
 * 前者解决“CAN 硬件刚收到数据，CPU暂时还没来得及取”；后者解决“ISR已经把硬件数据取出来了，但任务暂时还没来得及处理”。这其实就是两级缓冲。
 */
bool bsp_motor_can_init(void)
{
    /* 队列复制的是整个报文结构体，不是指向中断局部变量的指针（生命周期）。 */
    rx_queue = xQueueCreate(RX_QUEUE_LENGTH, sizeof(MotorCanPacket));
    return rx_queue != NULL;
}

bool bsp_motor_can_start(void)
{
    if (!rx_queue) return false;
    if (debug_motor_can.started[0]) return true;//启动CAN1,DONT RETRY

    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;             /* CAN1 使用第 0 组过滤器。 entry fifo0*/
    filter.SlaveStartFilterBank = 14;   /* 0..13 留给 CAN1；14..27 留给未来 CAN2。 */
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
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) goto failed;

    /* 从初始化状态进入通信状态；本工程已由你在 CubeMX 配成 Normal。
     * 此函数内部可能等待硬件状态，所以从任务调用，不放进临界区/中断。
     */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) goto failed;

    /* CubeMX 打开的是 NVIC；这里打开 CAN 外设内部的事件通知。
     * 两层都打开，接收事件才会进入 IRQ -> HAL -> 我们的回调。
     */





    //piece together a bitmask to entry interrupt
    uint32_t notifications = CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_ERROR | CAN_IT_BUSOFF |
        CAN_IT_ERROR_PASSIVE | CAN_IT_ERROR_WARNING | CAN_IT_LAST_ERROR_CODE;
//start IQR SOURCE
    if (HAL_CAN_ActivateNotification(&hcan1, notifications) != HAL_OK) {
        uint32_t error = HAL_CAN_GetError(&hcan1);
        (void)HAL_CAN_Stop(&hcan1);
        debug_motor_can.last_error[0] = error;
        debug_motor_can.start_failures[0]++;
        return false;
    }

    debug_motor_can.started[0] = 1; /* 启动成功，不代表已收到电机反馈。 */
    return true;

failed:
    /* 共用失败出口；goto 只用于集中记录启动错误，不参与正常接收流程。 */
    debug_motor_can.start_failures[0]++;
    debug_motor_can.last_error[0] = HAL_CAN_GetError(&hcan1);
    return false;
}

/* 任务调用：有报文就复制到 *packet，没有就立即返回 false。 */
bool bsp_motor_can_take(MotorCanPacket *packet)
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
    if (can != &hcan1 || !rx_queue) return;
    BaseType_t wake = pdFALSE;//这次中断往队列塞数据，不会唤醒更高优先级任务

    /* bxCAN 的硬件 FIFO 最多存 3 帧。一次最多取 3 帧，限制中断执行时间。
     * 中途又到达的数据仍可触发后续中断，不在这里无限处理。
     */
    for (unsigned n = 0; n < 3 && HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0); ++n) {
        CAN_RxHeaderTypeDef header;//a frame of CAN include ID IDE(BASE OR EX) RTR REMOTE DLC :LEN
        MotorCanPacket packet = {0};

        /* header 得到 ID/帧类型/长度；packet.data 得到最多 8 个数据字节。 */
        if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &header, packet.data) != HAL_OK) {//take away one frame
            debug_motor_can.errors[0]++;
            debug_motor_can.last_error[0] = HAL_CAN_GetError(can);
            break;//fail
        }
        debug_motor_can.rx_frames[0]++;
        /* DJI 电机反馈使用标准数据帧，长度必须为 8。 */
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8) {//11bit ID  RTR==DATA  DLC==8
            debug_motor_can.rejected[0]++;
            continue;//skip
        }


        //SUMMERIZING:
        packet.bus = 1; /* 本阶段仅启用你配置的 CAN1。 */
        packet.id = (uint16_t)header.StdId;
        packet.length = (uint8_t)header.DLC;
        packet.received_at = xTaskGetTickCountFromISR();

        /* CAN 同时接收多个电机，不能像单遥控器那样只覆盖一份最新报文。
         * 队列保留多帧；满时记录丢包，任务里用时间戳判断反馈是否过期。
         */
        //TODO:using send ,avoid cover
        if (xQueueSendFromISR(rx_queue, &packet, &wake) != pdPASS)
            debug_motor_can.queue_drops[0]++;//producer>consumer
    }
    /* 如果队列操作唤醒了更高优先级任务，请求退出中断时调度它。 */
    portYIELD_FROM_ISR(wake);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *can)
{
    if (can != &hcan1) return;
    /* 只记录错误；不要在这里打印或等待硬件重新初始化。 */
    debug_motor_can.errors[0]++;
    debug_motor_can.last_error[0] = HAL_CAN_GetError(can);
}

/* 不再定义 CAN1_RX0_IRQHandler / CAN1_SCE_IRQHandler：
 * 唯一入口保留在 CubeMX 生成的 Core/Src/stm32f4xx_it.c 中。
 */

/* CAN1/0x200 只有 ControlTask 使用，不与其他任务争用发送句柄。
 * 始终最多保留一条未完成的底盘指令；新指令到来时取消旧请求。
 * 禁止把历史非零电流积压在多个 mailbox，尤其不能让旧指令排在停机零帧前面。
 */
static uint32_t chassis_mailbox;
static bool chassis_mailbox_used;
volatile ChassisCanTxDebug debug_chassis_tx;

bool bsp_motor_can_ready(void)
{
    return debug_motor_can.started[0] &&
        HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_LISTENING &&
        !(hcan1.Instance->ESR & CAN_ESR_BOFF);
}

bool bsp_motor_can_send_chassis(const int16_t current_raw[4])
{
    if (!current_raw || !debug_motor_can.started[0]) return false;
    if (chassis_mailbox_used && HAL_CAN_IsTxMessagePending(&hcan1, chassis_mailbox)) {
        debug_chassis_tx.cancel_requests++;
        if (HAL_CAN_AbortTxRequest(&hcan1, chassis_mailbox) != HAL_OK) {
            debug_chassis_tx.errors++;
            return false;
        }
        /* 取消请求由硬件异步完成；不忙等，下一控制周期再尝试零帧。 */
        if (HAL_CAN_IsTxMessagePending(&hcan1, chassis_mailbox)) {
            debug_chassis_tx.busy++;
            return false;
        }
    }
    chassis_mailbox_used = false;
    if (!bsp_motor_can_ready()) return false;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        debug_chassis_tx.busy++;
        return false;
    }
    uint8_t data[8];
    chassis_pack_current(current_raw, data); /* 最后一道电流码值限幅 + 大端打包。 */
    CAN_TxHeaderTypeDef header = {0};
    header.StdId = 0x200;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;
    header.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &chassis_mailbox) != HAL_OK) {
        debug_chassis_tx.errors++;
        return false;
    }
    chassis_mailbox_used = true;
    debug_chassis_tx.queued++; /* 仅表示进入硬件 mailbox，不能称为电机已执行。 */
    return true;
}
