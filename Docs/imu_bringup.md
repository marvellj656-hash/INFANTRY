> 以下保留早期阶段记录，不代表当前功能范围。最新操作、接口与验证请看 [本次上车指南](bringup_guide.md) 和 [迁移记录](migration_status.md)。

> 2026-09-26：接口已更新。当前接口与验证范围以 [架构指南](architecture.md) 为准。

# 本轮云台迁移：BMI088 采集准备

当前完成 Bsp/imu/bsp_imu.c 的HAL SPI事务、Modules/imu/bmi088.c 的识别/配置/换算，以及 Task/ins/ins_task.c 采集任务。未修改 .ioc 和 Core 生成代码，外设由用户在 CubeMX 配置。未接温控、零偏标定和原QuaternionEKF，未开启云台输出，不能视为完整云台迁移。

CubeMX：SPI1 Full-Duplex Master，8bit，MSB first，Software NSS，CPOL High / CPHA 2 Edge，Prescaler16（APB2=84MHz时5.25MHz）。PB3 SCK、PB4 MISO、PA7 MOSI；PA4加速度片选和PB0陀螺片选，GPIO Push-Pull Output，No Pull，初始High。SYS Debug设Serial Wire，释放JTAG引脚。先不启用SPI DMA/中断。将外设初始化分别生成 .c/.h，保证 spi.c/spi.h 及 MX_SPI1_Init 已生成并由main调用。

原ioc的Prescaler8在84MHz时得到10.5MHz，超过Bosch列出的SPI最大10MHz，因此改为16。
数据手册：https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf

配置后生成代码，重编译烧录，同一个新ELF加载到LinkScope。
直接观察 bmi088.accel_id=30(0x1e)、gyro_id=15(0x0f)、initialized/valid、samples、accel[0..2]、gyro[0..2]、temperature_c。尚未配置SPI时initialized=false是预期行为，每秒尝试一次，不伪造有效数据。加速度为m/s²，角速度rad/s；静止时三轴加速度合模应接近9.81，陀螺仍可能有本机零偏。

本阶段只有500Hz轮询采样，不包含数据就绪同步、自检、安装方向变换或有效姿态发布，不可接到云台控制。BMI088芯片ID回读能识别常见断线，不等于完整健康检测。

七组native测试通过，包括HAL SPI启用路径的dummy字节、片选释放、寄存器回读、正负温度、单位换算和错误恢复。无SPI配置的ARM构建通过；待用户生成SPI1后验证硬件分支的ARM构建和实机数据。