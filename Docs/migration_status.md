# 本次迁移与验证记录

日期：2026-09-26。工程：OMNI_freeRTOS/OMNI_jpl。
分支：OMNI/jpl_INFANTRY/freeRTOS；当前 HEAD：41d42af。
修改保留在工作区，未新增提交、未推送、未烧录。原来已经暂存/未暂存的修改没有 reset 或 stash。

## 已接入

| 项目 | 当前状态 |
| --- | --- |
| 电机 | 静态对象、语义 ID、句柄、自动分组、只读快照、指令/反馈超时 |
| CAN | 设备任务统一收发，应用任务不拼报文 |
| 底盘 | 保留原运动学/PID/功限/归零联锁，拆分 task/control |
| 云台 | 替换未完成草稿，归中、超时锁定、惯性双环、限位、手动/AUTO接口 |
| 摩擦轮 | 双速度环、原旋向、斜坡、到速检查 |
| 拨弹 | 单发、三连发、连发、反转、堵转退弹、恢复松扳机 |
| INS | BMI088 + 旧四元数 EKF，静止零偏、姿态快照、失效重置 |
| 目录 | Algorithm/Bsp/Modules/Task 分模块子目录；App 已迁到 Task |
| 资源 | FreeRTOS 堆 32 KB，配置与 .ioc 同步 |
| 文档 | 架构指南、接口速查、遥控器操作图、联调路线、LinkScope 七组快捷预置 |
| NUC | USB FS CDC、BCP 字节协议、目标快照、独立超时、姿态回传 |
| 裁判 | USART6 循环 DMA/IDLE、CRC、逐类新鲜度、交互 TX FIFO |
| 客户端 UI | 原九元素、ADD/MODIFY、真实客户端 ID、R 键及重连重画 |

## 验证命令与结果

在工程根目录执行：

```powershell
./Tests/run_native_tests.ps1
cmake --build build-arm-debug -j 8
arm-none-eabi-size build-arm-debug/OMNI_jpl.elf
```

15 组主机测试全部通过，编译使用 C11、-Wall、-Wextra、-Werror：

1. aux_control：云台归中/超时/模式/限位，摩擦轮到速，单发去重、三连发、连发、退弹、失联恢复。
2. imu：ACC dummy byte、GYRO 格式、失败释放片选、ID/配置读回、单位与恢复。
3. pid：独立实例、限幅、死区、dt、复位。
4. chassis：轮序、遥控范围、斜坡、停机/归零联锁、功率模型、协议打包。
5. can：双 CAN、启动独立性、过滤器、ISR 限量、队满、撤销旧帧、bus-off。
6. remote：DBUS 解码、非法输入、DMA 回绕、IDLE、UART 重启。
7. protocol：大小端、负数反馈、非法帧。
8. modules：重复注册、句柄、快照、批量输出、限值、超时/回绕、发送失败、只读、多圈角重建。
9. attitude：静止零偏、倾斜初始化、yaw 正负与积分、无效 dt、EKF 复位。
10. cmd：九拨杆组合、四通道回中、实际底盘 spin 启动、触发、键盘与 AUTO 基准/重连。
11. transmission：旧固定向量、半包/粘包/错包、100ms目标与500ms心跳独立超时、代次与时间回绕。
12. nuc_usb：初始化失败重试、静态类对象、TX复制/busy、RX复制、溢出/重连作废旧包。
13. referee：CRC固定向量、流恢复、业务长度、快照、身份代次、超时/时间回绕。
14. referee_bsp：DMA边界、迟到HT/TC、FIFO溢出/恢复、TX缓冲生命周期与超时。
15. referee_ui：九布局、位宽、小端、客户端ID、ADD/MODIFY、忙重试、过期与热量显示。

其中 IMU mock 已模拟 GYRO_BANDWIDTH 固定 bit7=1：写0x01正常读0x81，验证掩码比较能通过，而不是假设整字节相等。
LinkScope 七组 INI 均通过 -CheckOnly；使用安装包内 GDB 离线加载本次 ELF，71 个不同变量表达式全部解析成功。没有连接板子、没有启动采样。

ARM Debug 构建成功；控制任务不含旧 RT-Thread 调用或 bsp_can 依赖；git diff --check 通过。
Flash 链接占用 123584 B（11.79%）；RAM 区链接占用 53184 B（40.58%）；CCMRAM 未使用。
这些数值不等于运行期堆余量或任务栈高水位，后两者仍需上板测量。
产物：build-arm-debug/OMNI_jpl.elf。

## 保留边界

- 四轮控制顺序保持不变，FL/FR/RL/RR 的物理对应待确认。
- NUC 与裁判通信、裁判 UI 已移植；裁判热量约束、底盘动态功率策略、云台跟随底盘、弹仓盖舵机及 IMU 温控没有在本次新增。裁判快照目前被 NUC 队色和 UI 消费。
- AUTO 由鼠标右键+新鲜NUC角度许可；拨轮负向已映射手动反转。三连发保留算法接口，不占用本次遥控挡位。
- 无磁力计时航向会漂移；云台安装方向、机械中心、符号和 PID 需要实测。
- 主机测试不模拟抢占、真实 CAN 仲裁、电机惯量或真实弹道；没有声明整车已验证。

原文件备份在 .migration-backup/handle-api-20260926-183004；后续改动前内容在
.migration-backup/handle-api-20260926-184042。不要直接整体覆盖备份来回退，否则会混淆新旧目录；
需要回退时按 changes.txt 和 Git diff 逐项检查。

## 2026-09-26 上车问题修复与配置合并

- 修复 BMI088 陀螺仪带宽寄存器只读位造成的初始化误失败；INS无效→云台未归中→摩擦轮被阻断的链路保留保护，并增加原因变量。
- cmd 的三个设备命令统一从纯映射输出；CH3不再同时转底盘；新增SPIN、拨轮反转、AUTO接线、UI重画。
- 保留用户生成的 USART6 PG14/PG9、DMA2 Stream1 Channel5循环接收、中断优先级5与启动调用。主控没有被烧录或复位。
- 基于用户最新配置，只定点修改 PLLQ=7 和 FreeRTOS堆32768，并同步 .ioc；HSE12MHz/M6/N168/P2 保持 CPU168MHz，USB48MHz。
- CubeMX这次再生成曾把堆回退到15360。继续编辑前应重新打开磁盘上的 .ioc，检查 TOTAL_HEAP_SIZE=32768 与 PLLQ=7，避免旧窗口缓存再次覆盖。USB Device/CDC中间件由工程维护；USB PCD硬件句柄、MSP和唯一OTG_FS中断由CubeMX生成。
- 本次合并前文件逐项保存在 `.migration-backup/controls-links-20260926-211144`；changes.txt 区分新增和修改。后续文档补充另有同前缀时间戳备份。
- 所有测试是主机替身或离线编译，USB真实枚举/电气、裁判终端显示、CAN方向与机械中心仍需上车验证。固件测试指南：[bringup_guide.md](bringup_guide.md)。

## USB 时钟配置再生成后的合并

用户的 PLLQ=7 / USB48MHz 配置已核对并保留。新生成的 usb_otg.c 纳入硬件所有权，Bsp/nuc 改用同一个 hpcd_USB_OTG_FS；删除 BSP 中重复的 MSP/IRQ 和根 CMake 中重复的 HAL 源文件。补齐 OTG_FS 优先级5，并在 .ioc 同步；恢复被生成器回退的32KB RTOS堆。主机应用算法未改变，需重新 ARM 构建和检查唯一符号，真实 USB 枚举仍需上板。

本轮 USB 合并验证：ARM Debug 重新构建成功，FLASH 123664 B、RAM 53192 B；ELF 中 hpcd_USB_OTG_FS、HAL_PCD_MspInit、HAL_PCD_MspDeInit、OTG_FS_IRQHandler 各恰好一份，旧 nuc_pcd 已不存在。compile_commands 中三份 HAL USB 源码各只编译一次。此前15组主机测试覆盖的应用算法未改动，本轮没有重复宣称硬件枚举已通过。未烧录、未提交；备份位于 .migration-backup/usb-cubemx-20260926-213707。