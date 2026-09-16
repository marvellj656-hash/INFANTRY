# 底盘低速闭环试转（2026-09-16）

当前代码可以发送实际电机电流。尚未由助手连接硬件或烧录验证。

## 本轮调用链

RemoteTask -> chassis_target(CH1, CH2, CH3) -> command_queue -> ControlTask（2 ms）
-> motor_feedback_get_chassis（原子快照）-> chassis_step（斜坡、解算、PID、估算功率限制）
-> bsp_motor_can_send_chassis -> 标准数据帧 0x200 -> CAN1 上 C620 ID1..4。

CAN1 使用 CubeMX 的 hcan1、PD0/PD1、1 Mbit/s。不修改 .ioc 或生成的初始化/中断文件。
没有调用云台状态或回中函数，不给 Yaw、Pitch 或发射电机发送控制命令。

## 首次试转

1. 轮子架空，遥控器 SW2 上档，CH1/CH2/CH3 归零。烧录 build-arm-debug/OMNI_jpl.elf。
2. CLion 不保持暂停状态；使用 LinkScope 时避免两个 OpenOCD 同时抢 ST-Link。LinkScope 必须使用同一 ELF。
3. 加载 Docs/LinkScope-drive.ini。debug_motor[0..3].frames 应增长；CAN1 started[0] 应为 1。
4. SW2 中档，摇杆保持零位一次。debug_chassis.armed=1、fault_flags=0 后轻推一个轴。
5. CH1 对应车体 vx，CH2 对应 vy，CH3 对应旋转 wz；CH4 和 SW1 本阶段不参与底盘控制。
6. 先核对 target_rpm 与 measured_rpm 的符号和实际轮序，再验证横移、前后、旋转。新旧轮序均为 0x201..0x204；机械前后左右须现场确认。
7. SW2 上档或下档均发送零电流；遥控数据超过 100 ms、任一底盘电机反馈超过 20 ms、周期异常或 CAN 发送失败均清 PID 和斜坡。零电流是释放驱动力，不是机械急刹。
8. 故障恢复/重新使能后要求摇杆归零一次再允许动作，避免带着旧指令突然恢复；这与云台归中无关。

## 当前限制与可调位置

Algorithm/chassis_config.h 集中管理参数：最大平移 300 mm/s，最大旋转 0.5 rad/s，平移加速度 500 mm/s²，角加速度 1 rad/s²，转子速度上限 1000 RPM，电流码值 ±2000（约 ±2.44 A）。保持整个摇杆行程的比例，不在末端把原 2000 mm/s 的指令硬截成 300。

原工程轮周长 471 mm、减速比 19、底盘半径 230 mm 和 omni_calc 符号沿用。PID 沿用 Kp=6、Ki=0.2、Kd=0.0001，采用实际 dt、梯形积分、对测量求微分，加上输出/功率饱和防积分累积。

## 功率控制的真实边界

当前有四轮统一缩放的估算功率限制，默认预算 30 W；电流为原始码值 -> A，实测转子速度 RPM -> rad/s，再计算机械功率和电阻损耗。不同轮子的负功率不抵扣其他轮子的正功率，采用保守模型。参数未实测标定，不能保证真实输入功率始终小于 30 W。

裁判系统实时限功率、能量缓冲控制、超级电容协议/充放电协调和 RLS 在线辨识尚未接入。当前固定预算仅用于低速联调，不等于完整功率控制方案，不可宣称整车移植完成。

## 故障位 fault_flags（可以相加）

| 值 | 含义 |
|---|---|
| 1 | SW2 不是中档，或遥控器过期 |
| 2 | 未收到四轮有效反馈，或任一轮反馈超过 20 ms |
| 4 | 控制间隔为 0 或超过 20 ms |
| 8 | CAN 尚未启动/Bus-Off/上周期或本周期发送失败 |
| 16 | 已具备运行条件，等待摇杆归零以使能 |

debug_chassis.current_raw 是计划提交的电流；debug_chassis_tx.queued 表示 HAL 已接受到邮箱，不代表电机已执行。debug_motor[i].speed_rpm 为实际反馈。
CAN 邮箱最多保留一条本任务的未完成指令，新指令替换旧请求；取消尚未完成时不忙等，下周期重试零帧。已经在总线上发送完成的报文无法撤回。

## 阅读顺序

1. Algorithm/chassis_config.h：单位和限制。
2. Algorithm/chassis_control.c：先读 chassis_target、chassis_inverse，再读 chassis_step、chassis_power_limit。
3. App/robot_app.c 的 ControlTask：输入有效性、周期、调用顺序、故障时发送零值。
4. Bsp/bsp_motor_can.c 末尾：CAN 帧打包和邮箱更新。

## CubeMX 生成后 CLion 报红

先 Reload CMake Project，让新增的 can.c、包含路径和 HAL 宏重新加载。以 Build 输出第一条真实错误为准：
* 编译通过：通常是代码模型未刷新；仍异常再 Reset Cache and Reload Project，保留原 Debug 工具链和 build-arm-debug 路径。
* 找不到头文件：检查生成结果和目标 include 路径。
* 重复定义 hcan1 / IRQ：不能同时保留手写硬件入口和 CubeMX 入口。
* PC/ESP 编译器出现在日志：检查 Debug 使用 cmake/gcc-arm-none-eabi.cmake。

不要为了编辑器红线反复增删 CAN 配置，生成代码也不代表它自动开始 CAN 通信。
