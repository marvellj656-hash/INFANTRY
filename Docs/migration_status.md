# 整车迁移对照与未完成项

日期：2026-09-16。依据旧工程 rtconfig.h、src/task/robot.c 和实际源文件。编译通过不等于实车通过；下列未完成项不能静默删除。

| 原功能/位置 | 新实现/状态 | 剩余工作 |
|---|---|---|
| rc/dbus + cmd 遥控 | dbus_protocol、bsp_remote_uart、RemoteTask 已接入 | 板上验证持续更新/失联；CH1/2/3 已用于底盘 |
| rc/keyboard | 解码保留键鼠字段 | 键鼠模式、键位、鼠标云台控制尚未接入 |
| CAN1 + DJI feedback | CubeMX can.c、bsp_motor_can、motor_feedback | 板上验证四轮方向/ID |
| chassis omni_calc | chassis_inverse 沿用几何/轮序 | 现场确认轮序与符号 |
| chassis 速度 PID | chassis_control，2 ms 周期、限流、反积分饱和 | 实车调参、长期运行验证 |
| chassis 斜坡 | 实际 dt、合速度及角速度限加速度 | 起步/反向手感验证 |
| 底盘独立运行 | 去掉云台回中依赖，车体坐标手动控制 | 云台跟随、无头、小陀螺、高低速/Boost 模式尚未移植 |
| chassis 功率限制 | 当前有未标定模型的四轮统一限功率 | 实测模型标定、裁判动态预算、缓冲能量、RLS 辨识、超电协调尚未完成 |
| supercap_task | 原协议已定位，CAN1 上可接收原始帧 | 协议解码/发送、故障回退、充放电/Boost 协调待移植 |
| referee_task | 未移植 | CubeMX USART6 + DMA、CRC/帧解析、裁判状态、热量/功率、各命令 |
| referee UI_Print | 未移植 | UI 编码/调度/发送和 DMA TX 归属 |
| ins_task + BMI088 | 未移植 | CubeMX SPI1/CS/温控 PWM、传感器标定/姿态融合 |
| IST8310 磁力计 | 未移植 | I2C 接口/引脚、数据接入 |
| gimbal_task | 仅 CAN1 Yaw 反馈可见 | CAN2、编码器/IMU 双环、限位、归中、自瞄、失联 |
| shoot_task | 未移植 | CAN2 摩擦轮/拨弹、热量限制、卡弹处理、弹仓 PWM |
| transmission_task | 未移植 | USB CDC、收发协议、心跳、重连、目标数据 |
| motor_task 分组发送 | CAN1 底盘 0x200 单一发送入口 | CAN2/GM6020 各组仲裁，避免多任务重复写同一分组 |
| msg 发布订阅 | 当前用 FreeRTOS 队列/反馈快照替代 | 其余整车话题在接入各模块时逐项映射 |
| INA226/其他电机与示例 | 原目录存在，但未在已检查配置中启用 INA226 任务/HT/LK | 保留记录，确定整车需求后移植或明确标为备用 |

## 外设配置分工

用户操作 CubeMX，助手提供参数并衔接生成结果。当前已配置 CAN1、USART3 RX DMA、TIM6 HAL 时基。后续需要 CAN2、SPI1、I2C/软件 I2C、USART6、USB CDC 和相应 PWM；不能把这些尚未配置的硬件功能标记为运行完成。

下一硬件验收点：低速架空试转，确认四个电机的目标 RPM/实测 RPM 同号、SW2 停机、失联/反馈超时停机。此阶段不是整车最终交付。
