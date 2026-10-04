# GM6020 云台 LQR / ESO 学习与数据实验

这一步的验收：能解释 LQR 与 ESO，确认电流命令路径，导出可追溯的 PID 基线数据，并在 MATLAB 中比较跟踪误差与拟合模型。尚无实车数据时不生成上车增益。

## 1. 当前工程与原方案

实际车载工程：`E:/LEARN_CODE/OMNI_INFANTRY/OMNI_freeRTOS/OMNI_jpl`；VS Code 工作区：其上一层 `OMNI_freeRTOS`。实验分支：`JPLcodex/gimbal-lqr-eso-study`。原分支 `main` 的基点为 `d4188d1`。用户已有的 `Task/chassis/chassis_config.h`、`Task/gimbal/gimbal_config.h` 修改保留且不纳入本次提交。

来源：[Combat 战队原文](https://bbs.robomaster.com/article/1935544)、[控制器仓库](https://github.com/LamdaDay/YAW_Auto_Controller)。本次阅读固定在 `665c5b4ab1067d6cb63122c120822f27502953e5`，未将其源码复制到本工程。原文声明 CC BY-NC-SA 4.0；后续移植时核对代码的授权与署名要求。

原方案的链路是：连续参考角度/速度/加速度 → LQR 反馈与前馈 → 可选 ESO 补偿 → 偏置补偿 → 力矩限幅与斜率限制。它主要针对 yaw；其作者车辆使用 DM4310。当前工程的视觉接口只给 `auto_yaw_deg`、`auto_pitch_deg`，没有连续参考速度、加速度，需另外处理时间戳、角度绕回和参考轨迹。直接对低频视觉角度连续差分两次容易放大噪声，不能替代轨迹设计。

软件检查确认：

| 项目 | Yaw | Pitch |
|---|---|---|
| 型号配置 | DJI_GM6020_CURRENT | DJI_GM6020_CURRENT |
| 总线 / 电调 ID | CAN1 / 3 | CAN2 / 4 |
| 反馈 ID | 0x207 | 0x208 |
| 电流命令帧 | 0x1FE | 0x1FE（另一条 CAN） |
| 当前指令限流 | 1.5 A，码值 8192 | 1.5 A，码值 8192 |
| 反馈角与角速度 | deg / rad/s | deg / rad/s |
| 控制输出方向 | 相对惯性角速度取反 | 保留正号 |

依据用户提供的 `6020.pdf` 第 6–7 印刷页（PDF 第 7–8 页）：电流控制 ID 1–4 用 0x1FE、5–7 用 0x2FE；命令 ±16384 对应 ±3 A。要求电机固件 ≥1.0.11.2、RoboMaster Assistant ≥2.7，且打开电流环开关。上述命令换算不能直接当作反馈报文电流的单位；手册这两页未给反馈码到 A 的标定。带 PWM 接口不足以单独确认电机型号或电流模式，仍需看铭牌/Assistant 设置。

控制输出换算是 `i_cmd_A = raw * 3 / 16384`，力矩换算还需 `tau = Kt * i`。**电流环已经改好，不代表已经知道 Kt 或云台惯量。**

## 2. 从串级 PID 理解 LQR

角度环把角度误差变成希望达到的角速度，速度环把速度误差变成电流，电调内部再闭合电流环。LQR 可以保留电调内部电流环，替换 MCU 上的角度/速度反馈计算。

先把单轴看作一个惯量：

`J * theta_ddot = tau - B * theta_dot + d`

- theta：轴角度，rad；theta_dot = omega：角速度，rad/s。
- J：等效转动惯量，kg·m²；B：黏性阻尼，N·m·s/rad。
- tau：电机在该轴产生的力矩，N·m；d：未建模的总扰动力矩。

定义 `e_theta = theta - theta_ref`、`e_omega = omega - omega_ref`。LQR 的反馈是：

`tau_fb = -K_theta * e_theta - K_omega * e_omega`

它在这个线性模型上最小化“未来角度误差平方、速度误差平方、控制力矩平方”的加权积分。Q 给误差权重，R 给用力权重。模型和单位固定后，增大角度权重通常会追得更紧；提高速度权重会改变阻尼；提高 R 通常会使动作柔和。最后仍需检查采样后的稳定性、噪声和电流饱和。[MathWorks LQR 文档](https://www.mathworks.com/help/control/ref/lti.lqr.html)

它不是会自己学习参数的 AI；MCU 中最终只需进行少量乘加。求增益的工作可以离线在 MATLAB 做。最优性只针对选定的线性模型与代价函数，经典 LQR 不直接处理硬限流、机械限位和时延，不能保证比已经调好的 PID 更好。

对于这类二阶轴，LQR 反馈从形式上接近角度与速度反馈。它的价值在于用模型和权重系统地设计取舍，并与参考速度、加速度前馈结合。

## 3. 为什么前馈能提高跟踪

反馈要等误差出现才能纠正；前馈根据参考轨迹提前提供所需力矩：

`tau_ff = J * alpha_ref + B * omega_ref`

例如目标即将加速，先给克服惯量需要的力矩；目标匀速转动，先补黏性阻尼。只有参考速度/加速度可靠、符号和模型正确时才有帮助。源代码另有平滑库仑摩擦前馈；初次实验应逐项打开，避免和 ESO/积分重复补偿。

## 4. ESO 是怎样估计扰动的

观测器一边预测云台会怎样运动，一边比较预测角度和测得角度。持续出现的预测误差用于修正角度、速度与一个额外的“扰动状态”，所以叫扩张状态观测器。

可写成 `omega_dot = -(B/J)*omega + tau/J + f`。三阶 ESO 中 z1 估计角度、z2 估计速度、z3 估计总扰动加速度 f。若估计有效，可加 `tau_eso = -J*z3` 抵消它。[MathWorks ESO 文档](https://www.mathworks.com/help/slcontrol/ug/extendedstateobserver.html)

注意源控制器的 z3 单位是 rad/s²，**不是 N·m**。ESO 使用上一周期最终限幅后的命令；用未限幅的请求会误以为电机已输出了实际没有输出的力矩。

带宽高，扰动估计通常更快，也更容易受到角度噪声影响。带宽低，曲线更平滑但补偿慢。角度绕回、反馈不新鲜、时序不一致和参数符号错误也会被当成扰动。先只观察 ESO，再小幅打开补偿并限制补偿力矩，比直接开满更容易判断效果。

原代码还含积分和力矩偏置自适应；这些不是基础 LQR 必需项。原代码的积分只有积分量限幅，不能当成完整的饱和抗积分饱和策略；初次接入建议积分与偏置关闭。其 dt 越界会回退到 1ms，NaN 输入没有统一拒绝路径，接入时需保留本工程的有限值、故障和时间检查。ESO 显式 Euler 离散化也需要检验实际 dt 与带宽。

## 5. 能不能用于全向轮云台

Yaw 可以先做单轴实验。全向轮底盘不会阻止使用这种控制器，但底盘旋转、加减速、倾斜与不同 pitch 姿态会改变 yaw 的等效负载与扰动，需要最后加回这些工况验证。

Pitch 需要另建模型：`Jp*theta_ddot = tau - Bp*omega - gravity(theta) + d`。先测多个俯仰姿态的维持电流，拟合重力项，再分别辨识局部动态；不能把 yaw 的 J、B、Kt、增益复制给 pitch。还需核对惯性角度导数与选用陀螺分量的关系，尤其在大仰角和底盘运动时。

## 6. EmberProbe 0.8.0 导出 CSV

1. 在 VS Code 打开 `E:/LEARN_CODE/OMNI_INFANTRY/OMNI_freeRTOS`。
2. 保持“安装 Agent Skills”开启。它让助手通过本地 Bridge 操作插件；无需手动改扩展存储。
3. 选择与**当前片上程序完全匹配**的 ELF；现在配置是 `OMNI_jpl/build-arm-debug/OMNI_jpl.elf`。选择 `stlink-v2.cfg`、`stm32f4x.cfg`。
4. 在“实时读写”中添加下表变量，再点“开始”和“图表”。也可用命令面板的 `EmberProbe: 实时变量查看` 打开图表。
5. 先记录静止片段和小幅慢速运动。默认请求 30Hz；可尝试 100Hz，但看实际采样 Hz、间隔与丢样，不把设置值当作测量值。
6. 图表点“导出 CSV”。优先选“完整采样归档”，选本轮时间范围和变量。若冻结了图表，确认是否默认选了“冻结快照”，避免误以为导出了全部实验。
7. 保存到 `OMNI_jpl/tools/gimbal_lab/data/`，例如 `yaw_pid_baseline_01.csv`。复制 `session_template.json` 并填写固件、参数、轨迹、pitch 姿态和底盘状态。保留原始 CSV。

| 变量 | 作用 / 单位 |
|---|---|
| debug_gimbal.yaw_target_deg / pitch_target_deg | 目标角，deg；INIT 时是编码器参考，不能与惯性角混用 |
| debug_ins.yaw_deg / pitch_deg | 当前姿态，deg；旧固件已有 |
| debug_gimbal.yaw_output / pitch_output | 提交给电机层的最终码值，当前模式用 raw*3/16384 得命令 A |
| debug_motor[4/5].queued_output_raw | HAL 接受的最近输出，用于核对发送链路；不证明电调已执行 |
| debug_motor[4/5].current_raw | 反馈电流原始码；先不套用命令换算 |
| debug_gimbal.mode / faults | 只比较正常运行同一模式的片段；2=手动，3=自瞄 |
| debug_transmission.age_ms / aim_valid | 区分视觉包陈旧与控制器跟踪慢 |

下一步拟在同一云台任务迭代中追加记录：`sample_time_ms`、`control_dt_s`、`yaw_angle_deg`、`pitch_angle_deg`、`yaw_rate_rad_s`、`pitch_rate_rad_s`。当前版本尚未增加这些字段；必须先实现、编译并烧录后才能使用。设计时应将字段追加到旧结构末尾，保持原字段布局。

SWD 对多个字段的读取仍不是原子快照，电脑时间也是轮询时间。MCU 时间戳只能帮助发现重复/跳过样本，不能自动消除跨周期读取。用于精确动态辨识时，下一步实现 MCU 周期内同步记录的缓冲或二进制串口输出，记录序号、时间戳、角度、角速度、最终命令、反馈与限幅标志，再整体传回。

本机版 EmberProbe 的目标采样设置上限为 200Hz；快动态/高频振动需更高频的 MCU 记录。作为经验起点，采样频率至少达到关心动态频率的 10–20 倍，并留意抗混叠和实际间隔。1kHz 控制任务不等于电脑录到了 1kHz；30Hz 对 5Hz 正弦只有约6点/周期，不能据此精确判断高速性能。

本机 Agent 导出命令（只导出已有图表归档，不连接探针）：

```powershell
./OMNI_jpl/tools/gimbal_lab/Export-GimbalCsv.ps1 -Axis yaw -LastSeconds 30
```

底层采用 EmberProbe 自带 `mcu-variables/scripts/read.js --export-csv`。CSV 归档是 `time`（ISO UTC 时间）加带类型的变量列；不要删除表头中的类型再猜映射。[EmberProbe 项目文档](https://github.com/BakeSheep/EmberProbe-MCU-Flash-Debug)

## 7. LinkScope 导出

保持同一时刻只有一个软件占用 ST-Link。先停止 EmberProbe 的采样并释放连接，再使用 LinkScope。

导入工程已有的 `Docs/LinkScope/gimbal_current.ini`，核对 ELF、ST-Link 和目标，添加目标角、测得角及角速度。点“连接”，采完后在工具菜单选择“导出数据 / Export Data”保存 CSV。不同版本时间表头不同，第一次导出先检查表头与单位，再写 mapping。[LinkScope 作者说明](https://github.com/Skythinker616/LinkScope)

已有预设默认 20Hz，适合检查模式与低频电流响应；模型辨识按实际采样能力另做同步采集。LinkScope 不烧录固件，换 ELF 需重新连接并确保它匹配片上程序。

## 8. MATLAB 的用法

在 MATLAB：

```matlab
addpath('E:/LEARN_CODE/OMNI_INFANTRY/OMNI_freeRTOS/OMNI_jpl/tools/gimbal_lab');
cd('E:/LEARN_CODE/OMNI_INFANTRY/OMNI_freeRTOS/OMNI_jpl/tools/gimbal_lab');
gimbal_lqr_eso_demo;  % 合成模型教学：反馈、前馈、ESO，不能作为实车成绩
gimbal_lab_selftest; % 离线自检，不连接机器人
```

这些工具使用基础 MATLAB，无须安装 Simulink 或额外工具箱。教学模型用二阶轴的解析连续 LQR 解，同时检查采样后的极点；它不等同于原作者控制器的逐行复现。

导入例子（列名必须换成你 CSV 中的完整实际表头）：

```matlab
mapping.timeColumn = 'time';
mapping.timeUnit = 'iso8601';  % LinkScope 根据实际表头换成 s/ms 等
mapping.signals.theta_rad = struct('column','debug_ins.yaw_deg:f32','scale',pi/180);
mapping.signals.theta_ref_rad = struct('column','debug_gimbal.yaw_target_deg:f32','scale',pi/180);
% yaw 输出方向与惯性角正方向相反，所以命令归一化时取负号：
mapping.signals.u_cmd = struct('column','debug_gimbal.yaw_output:i32','scale',-3/16384);
gimbal_import_csv('data/yaw_pid_baseline_01.csv',mapping,'data/yaw_normalized.csv');
gimbal_analyze_csv('data/yaw_normalized.csv','results/yaw_pid_baseline_01');
```

测量角与目标角必须在同一参考系；先截取 GYRO 或 AUTO 的有效片段再分析。旧固件的姿态与目标在不同任务中发布，不能据此保证同步。

导入器保留空样本，不插值，不按“设定频率”伪造时间；时间重复/倒退会拒绝，需先分开重启与不同会话。分析器输出 RMSE、95%绝对误差、最大误差、偏差、采样间隔与丢样提示。传入 `struct('frequencyHz',1)` 才做单频幅值/相位估计；等效相位延迟不代表纯通信时延。

完成同步采集后，归一化 CSV 加入 `omega_rad_s`，可用：

```matlab
model = gimbal_fit_axis('data/yaw_normalized.csv');
% 只有实测 Kt 后才传入 torquePerCommandUnit（输入为A时它就是 N*m/A）
```

拟合用速度增量的积分回归，避免直接对陀螺数据差分产生角加速度噪声。未知 Kt 时只输出命令到加速度增益与相对阻尼，不输出虚假的 kg·m²。使用前70%拟合、后30%验证；同时检查激励与矩阵条件数。闭环数据的普通最小二乘会受反馈噪声偏置，结果是初步等效模型，需不同轨迹/独立实验复核。命令模型不一定能把驱动器时延“吸收”成固定 J/B，快动态仍要检查输入输出时延。

## 9. 每轮实验只解决一个问题

1. **基线**：先录现有 PID，在相同机械结构、限流、姿态与轨迹下重复至少3轮。机械允许时从小幅1–3°、0.5Hz慢运动开始，幅度以不触碰限位为前提。
2. **辨识**：底盘静止，确认正方向与电流单位，做多个频率/双向运动的同步记录；避免单一恒速导致输入和速度共线。优先用命令电流建立有效模型，记录驱动器滞后和饱和。
3. **反馈**：只试 yaw LQR，关闭前馈、ESO补偿、积分、偏置；保留现有电流限幅、限位、掉线保护和模式切换规则。
4. **前馈**：参考速度/加速度已可靠时，单独启用惯量/阻尼前馈，重测相同轨迹。
5. **ESO**：先只观察估计，再限制幅度地启用补偿；确认无饱和追逐和高频抖动。
6. **Pitch**：先重力静态数据，再做独立控制设计。
7. **整车**：分别加入底盘平移、旋转、不同 pitch 姿态与真实视觉包。固定比较 RMSE、95%误差、相位、峰值电流、饱和比例，不能只比较一张最好看的波形。

助手负责读源码、记录版本、导出数据、查丢样/单位、拟合模型、仿真候选参数并给出下一轮可检验的改变；你负责实际机械状态与动作许可。在线写入 PID/控制变量需要先展示具体变量、单位、数值及插件写入计划，再按插件确认流程执行。不要强改故障许可来让实验跑起来。

当前只做离线工具和说明，未修改固件控制逻辑。待完成：同步采集字段/缓冲、实车基线 CSV、Kt 或有效电流模型、连续参考轨迹、实验参数选择、LQR/ESO 实车移植与对照验证。教学仿真不等于上车有效。
