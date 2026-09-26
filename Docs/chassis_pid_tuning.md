> 2026-09-26：接口已更新。当前接口与验证范围以 [架构指南](architecture.md) 为准。

# 底盘反馈与 PID 调参入口（2026-09-20）

本轮移植原 RTT `omni_get_speed` 的车体速度反馈。原函数将 `speed_rad` 用 RPM 系数换算；新函数 `omni_forward_kinematics` 明确接收转子 RPM。轮序、正负方向沿用原逆解：CAN1 0x201、0x202、0x203、0x204。

## 数据路径

遥控 CH1/CH2/CH3 -> 指令速度 -> 加速度斜坡 -> 四轮目标 RPM -> 各轮 PID -> 功率限制 -> 0x200 电流帧。
四轮实测 RPM -> 正解 -> measured_velocity（vx/vy 为 mm/s，wz 为 rad/s）。它是轮速估算，不是独立测量；轮子打滑时不能据此证明车在地面上直行。本轮未接 IMU、云台坐标或位置积分。

四轮独立 PID 参数位于 Task/chassis/chassis_config.h 的 CHASSIS_SPEED_PID_CONFIGS；四行依次对应 0x201~0x204。初值仍为原配置，未宣称已调好。pid_config_t 中 Ki 按秒积分，Kd 按秒微分，不要直接照搬忽略 dt 的其他 PID 数值。

## 本次上车步骤

1. 编译已通过；自行烧录 build-arm-debug/OMNI_jpl.elf，LinkScope 重新选择相同 ELF。本次新增状态改变了 RAM 布局，旧固件不能配新 ELF。
2. 先确认四轮 frames 都在增加、反馈新鲜、SW2=3，CH1/2/3 归中后 armed=1、fault_flags=0。偶发掉线尚未通过本次算法移植解决。
3. 低速单轴推进，再试横移和旋转。依次比较每轮 target_rpm 与 measured_rpm，确认同号、跟随。反馈或方向不对时先修对应链路。
4. 有持续振荡/稳态误差时再改该轮参数，每次只改一项，保留改变前后的曲线。先看 current_raw 是否饱和、power_scale 是否小于1：限流/限功率会影响跟随，不能直接归为PID问题。

## 直接观察控制状态

- debug_chassis.target_rpm[0]、debug_chassis.measured_rpm[0]、debug_chassis.current_raw[0]（索引0~3）
- chassis_controller.pid[0].Iout：真正参与控制的积分状态，没有额外副本。
- debug_chassis.measured_velocity_valid：必须为1，才能使用下一行的速度。
- debug_chassis.measured_velocity.vx / vy / wz：车体实际轮速解算。
- debug_chassis.power_scale、fault_flags：限制与停机原因。

chassis_controller 是 chassis_task.c 内的 static 对象。若直接输入名称解析有歧义，使用 'chassis_task.c'::chassis_controller.pid[0].Iout。

本轮不需要 CubeMX 配置；速度/加速度/电流上限、反馈超时、归中条件和功率预算保持原值。超级电容、裁判动态预算、RLS、IMU/云台/NUC 留在后续迁移清单。

验证：现有五组 native 测试通过；增加已知转子RPM的量纲检查、纯旋转、XY混合运动往返、停机时反馈仍有效及NaN反馈拒绝。ARM构建通过，未自动烧录，尚未实车调参。
## 2026-09-20 按原工程风格整理

控制入口在 Task/chassis/chassis_task.c，纯控制步骤在 Task/chassis/chassis_control.c/h。先读 task，再读 control，最后读 Algorithm/pid/pid.c。

PID从原 RTT 源码移植，保留作者标记、pid_obj_t/pid_config_t、INIT_PID_CONFIG 和八个优化标志：积分限幅、微分先行、梯形积分、比例先行、输出滤波、变速积分、微分滤波、堵转错误记录。默认仅开启原底盘使用的梯形积分、积分限幅、微分先行；不要把所有功能一股脑开启。滤波 RC 单位秒，变速积分 CoefA/CoefB 单位与误差相同。

必要接口替换：pid_register(config) -> pid_init(&object, &config)，不再动态分配；pid_calculate(pid, measure, ref) -> pid_calculate(pid, measure, ref, dt)，明确传实际周期秒。pid_clear 保留原来的系数，只清状态。

修正点：积分限幅临时量不再static共享；预测饱和时冻结同向积分；死区实际清Iout；首次调用不制造微分尖峰；无效周期返回零；变速积分避免CoefA=0除法。原比例先行标志未有对应实现，这次已补齐。堵转功能仍是可选错误记录，不等于可靠检测或自动停机。

直接观察 chassis_controller.pid[0~3] 的 Ref、Measure、Err、Pout、Iout、Dout、Output。不再使用之前的 integral 字段。

解除初次试车低上限：平移6000mm/s、旋转7rad/s、转子9000RPM、平移加速度3000mm/s²；角加速度3rad/s²。普通电流限幅8000码值，估算预算60W。PID仍使用6/0.2/0.0001，未声称实车调好。保护条件和掉线后归中规则保留。

六组本机测试及ARM构建通过，未烧录；重新烧录并在LinkScope重载同一ELF后使用新字段。无需CubeMX修改。
