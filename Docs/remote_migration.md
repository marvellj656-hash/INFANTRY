# 第一阶段：真实遥控器输入与二维速度斜坡

本阶段不发送 CAN 电机报文。硬件供电恢复正常后才上板验证。

## 数据流与阅读顺序

1. CubeMX 的 USART3_IRQHandler / DMA1_Stream1_IRQHandler 调用 HAL。
2. Bsp/bsp_remote_uart.c 是唯一的 HAL 接收事件与 UART 错误回调入口。
   只接受 &huart3；其他外设以后在这里明确分发，不能重复定义回调。
3. 循环 DMA 的 HT/TC 事件收集数据，IDLE 结束一段接收。
   只接受恰好 18 字节的段；错误长度立即发布无效事件。
   消息包含接收时刻，长度为 1 的队列按值复制最新帧。
4. App/remote/dbus_protocol.c 验证通道范围、拨杆和鼠标按键后一次性提交解析结果。
   协议无帧头/CRC，本实现靠 IDLE 定界，不声称能识别所有噪声伪帧。
5. App/robot_app.c 的 RemoteTask（原生优先级 3，每 10 ms）
   消费原始帧，判断在线与使能，发布二维速度指令。
6. 同文件 ControlTask（原生优先级 4，每 10 ms）再次检查原始接收时间，
   然后计算二维限加速度输出。只有这个任务修改输出。
7. Core/Src/freertos.c 只调用 robot_app_init；defaultTask 只闪灯。

## 本阶段模式与单位

- CH1 对应 vx，CH2 对应 vy，方向沿用旧代码轴名，车体正负方向待上板确认。
- CH3/CH4、sw1、鼠标键盘和滚轮解析保留，但不驱动云台或发射机构。
- sw2 中位 3 才使能；上位 1 和下位 2 均清零。本阶段不实现自瞄。
- 最大平移合速度 2000 mm/s，轴中心死区 ±5，合加速度 500 mm/s²。
  这些是初次迁移验证参数，不是最终整车性能参数。
- 超过 100 ms 没有有效帧：在线变 0、速度目标与斜坡输出直接归零。
- 无效长度、非法字段：撤销有效状态；新有效帧才能恢复。
- 串口错误：回调只记录原因，RemoteTask 中止并重新启动接收。
  接收重启清空旧队列，必须等新有效帧才重新使能。
- 控制周期超过 50 ms：清零本周期输出并重新对齐周期。
  未来接入电机时仍需独立的电机失能动作。

## 监视变量

- debug_remote.ch1/ch2/ch3/ch4、sw1/sw2：有效在线输入，否则清零/置上位。
- debug_remote.online、enabled、age_ms、valid_frames、invalid_frames。
  age_ms 为 UINT32_MAX 表示尚无有效状态，不是实际年龄。
- debug_remote_rx.idle_events/frames/length_errors：硬件收帧层。
- debug_remote_rx.uart_errors/last_uart_error/restart_attempts/start_failures。
- debug_control.target.vx/vy、output.vx/vy、enabled、cycles、timing_faults。
- 原教学变量 debug_target_mm_s / debug_ramped_mm_s 继续指向 X 轴值。
- debug_cmd_timeout 仅代表时间超限；拨杆失能请看 enabled。

以上 volatile 变量仅用于观察，整组字段读数不保证同一瞬间的一致快照。
任务间实际通过按值复制的队列通信，不通过调试变量通信。

## 上板验证

1. 正常供电、遥控器打开、sw2 上位：灯闪；valid_frames 增加，
   online=1，enabled=0，输出为零。
2. sw2 中位：CH1/CH2 随摇杆变化，分别驱动 vx/vy；归中目标为零，
   输出按斜坡减至零。斜推合速度不超过 2000。
3. 上位或下位：输出在下一次控制处理时直接清零。
4. 关闭遥控器或断开接收机：最后有效帧 100 ms 后，在接下来的控制周期
   内清零；恢复信号后应自动恢复在线。
5. 连续接收至少一分钟：帧数持续增加，跨 DMA 边界不应周期性失联。
6. 接收机断电再上电：检查能恢复，错误计数便于区分硬件/解析问题。

## 主机测试

在项目根目录，用 Windows 原生 GCC（不是 arm-none-eabi-gcc）执行：

    gcc -std=c11 -Wall -Wextra -Werror -ITests/support -IApp/remote -IAlgorithm -IBsp Tests/test_remote.c App/remote/dbus_protocol.c Algorithm/motion_command.c Bsp/bsp_remote_uart.c -o Tests/test_remote.exe
    Tests/test_remote.exe

测试真实解析/斜坡/BSP源文件；HAL和队列使用测试替身。
覆盖字段边界、非法输入、不同周期的斜坡、DMA回绕与重复IDLE、
错误长度、串口错误和重启失败重试。不能替代真实中断时序和整车验证。

## CubeMX 再生成

不修改 CubeMX 的 HAL 句柄、IRQ入口和生成的 cmake/stm32cubemx 文件。
freertos.c 接入代码位于 USER CODE 区。App/Bsp/Algorithm 在顶层 CMake 显式登记。
原来的模拟速度练习在 .migration-backup 中保留。
