# CAN 电机反馈接收

本阶段只接收反馈，不发送电流/电压控制报文。正常 CAN 模式会对合法帧应答 ACK。

## 文件阅读顺序

1. `Algorithm/dji_feedback.c`：将反馈的 8 字节解析成编码器、转速、电流原始值和温度。DJI 电机字节序是高字节在前，与 DBUS 的小端字段不同。
2. `Bsp/bsp_motor_can.c`：初始化硬件、过滤器、启动接收；中断复制报文到队列。
3. `App/motor_feedback.c`：任务取队列，按总线和 ID 路由，更新监视变量及 100 ms 离线标志。
4. `App/robot_app.c`：统一启动入口，增加 `motor_feedback_init()`。

## 原工程对应关系

原 `src/modules/motor/DJI_motor/dji_motor.c` 中的 `dji_motot_rx_callback()` 负责总线/ID 路由，`decode_dji_motor()` 负责解码。本版本将二者放进独立函数，不依赖 rt_device_t。

RTT 的接收通知/读取路径改为 HAL_CAN_RxFifo0MsgPendingCallback、HAL_CAN_GetRxMessage 和 xQueueSendFromISR；任务通过 xQueueReceive 取副本。每个电机的 RTT 定时器改成接收时间戳比较。

| debug_motor 下标 | 电机 | CAN | 反馈 ID |
|---|---|---|---|
| 0..3 | 底盘电机 1..4 | CAN1 | 0x201..0x204 |
| 4 | Yaw | CAN1 | 0x207 |
| 5 | Pitch | CAN2 | 0x208 |

`speed_rpm` 是转子转速，`current_raw` 是协议原始值，不能直接当作安培。暂未搬运滤波和多圈角度，先验证原始数据正确。

## 硬件配置归属

2026-09-14 已切换为用户通过 CubeMX 配置的 CAN1。Core/Src/can.c 独占 hcan1、GPIO、时钟和位时序初始化；Core/Src/stm32f4xx_it.c 独占 CAN1_RX0/CAN1_SCE IRQ。BSP 只负责软件队列、过滤器、Start/ActivateNotification 和 HAL 回调。根 CMake 不再手动添加 HAL CAN 驱动或使能宏，由生成文件管理。

CAN1: PD0 RX / PD1 TX，AF9。APB1 42 MHz，分频 3、BS1 9、BS2 4、SJW 2，1 Mbit/s；Normal、自动重传、自动 Bus-Off 恢复。NVIC 接收/错误优先级均为 5。过滤器 0，CAN2 分界为 14，标准数据帧进入 FIFO0。

本轮未启用 CAN2，也不在 BSP 偷开 CAN2。debug_motor_can 的下标 1、debug_motor[5]（Pitch）保持未启动/离线属于正常。下标 4 的 Yaw 只有实际接在 CAN1 且反馈 ID 为 0x207 时才更新。

接收队列长度 32，任务每 1 ms 最多处理 32 帧；满队列丢包可从 queue_drops 观察。以后再次生成 CAN1 配置，不需要在生成文件的 USER CODE 外手动补初始化。新增 CAN2 时再扩展 BSP 的句柄映射。

## 板上验证

烧录与 LinkScope 使用同一个 build-arm-debug/OMNI_jpl.elf。加载 Docs/LinkScope-motors.ini。

1. `debug_motor_can.started[0]` 为 1（下标 1 本阶段为 0）：仅证明外设启动完成。
2. `debug_motor_can.rx_frames[0]` 增长：对应总线收到报文。
3. `debug_motor[i].frames` 增长且 online 为 1：该电机有效反馈持续到达。
4. 在电机无驱动输出且机构允许的情况下缓慢转动，观察 ecd 改变；停止后 speed_rpm 接近零。
5. 停止某电机反馈后约 100 ms，online 变为 0；反馈值保留最后一次值，不能只看 ecd 判断在线。
6. queue_drops 应不增长；错误可看 errors / last_error，启动错误看 start_failures。

软件单元测试和固件编译不代替硬件 CAN 连接验证。遥控器任务继续运行。
