# LinkScope：点一下，选好一组变量

**2026-09-27 新固件：普通关闭摩擦轮现在会先闭环制动，短暂反向输出是预期行为；右上 disable 仍立即清零。新第5/8组必须配合这次固件与ELF。具体字段见文末最新章节。**

先用 `overview.ini`。它把“遥控有没有下命令 → IMU 有没有准备好 → 云台被谁挡住 → 发射被谁挡住”放在同一页，每组最多 12 个标量，默认全部勾选，20 Hz 采样。

## 最快的用法

1. 编译并烧录本次 `build-arm-debug/OMNI_jpl.elf` 对应的固件；LinkScope 选择的 ELF 必须来自这次构建。
2. 正常关闭已经打开的 LinkScope，双击本目录的 **`Open-LinkScope.cmd`**。
3. 在双击后弹出的黑色命令窗口里，看到 `Number [1]` 后输入 `1` 或直接回车，进入总览；需要看具体一层时再选择其他组。
4. LinkScope 打开后，确认变量能显示，再手动开始采样。脚本不会烧录、复位主控，也不会自动点击“开始采样”。

脚本默认使用已核对的安装位置 `E:/FOQQ/Linkscope/LinkScope-x86-V1.3.0/LinkScope.exe`。运行时先检查文件，再把原 `autosave.ini` 备份成同目录的 `autosave.before-preset-日期时间.ini`，然后换成所选预置。原来的设置可以从备份恢复。若 LinkScope 正在运行，脚本只提示先关闭，不终止任何进程；其他调试器占用 ST-Link 时应先正常结束那次调试。

也可以在 LinkScope 内手动导入本目录的 `.ini`；`EnableScope=true` 就是“已勾选”。手动导入时注意 INI 中的 ELF 路径；使用启动器时会自动改成当前工程的 ELF 路径。脚本没有依赖未经验证的 LinkScope 命令行导入参数。

安装位置或 ELF 路径不同，可以这样运行：

```powershell
.\Open-LinkScope.ps1 -Preset gimbal -AppDirectory 'E:/FOQQ/Linkscope/LinkScope-x86-V1.3.0' -ElfPath 'E:/LEARN_CODE/OMNI_INFANTRY/OMNI_freeRTOS/OMNI_jpl/build-arm-debug/OMNI_jpl.elf'
```

加 `-CheckOnly` 只检查文件路径和 INI 结构，不改配置、不启动程序。`.cmd` 也支持转发这些参数，例如 `Open-LinkScope.cmd -Preset imu`。不要拿旧 ELF 解释新固件的内存；编译后重新选择 ELF，避免 LinkScope 继续使用旧的符号缓存。

## 当前车上仍是旧固件：先用这个

**`Open-Shoot-Stop-Baseline.cmd`**：专门观察刚才上车的 `pre-OMNI9_26` 固件。自动选择已备份的旧 ELF，不需要烧录，也不用输入编号。正常关闭 LinkScope 后再双击，手动开始采样。

这页先看 `friction/enabled → raw[0/1] → speed_rpm`。`online` 必须为 1，`frames` 必须持续增加，否则显示的转速可能只是最后一次反馈；`debug_motor_task_cycles` 应持续增加，停止增长说明电机服务没有继续执行。没有新增 CAN 输出字段，因为旧固件里不存在这些字段。

用户已观察物理左摩擦绿灯每秒 2 次、右摩擦每秒 1 次，与现有代码声明相反。当前 `debug_motor[7]` 实际按 CAN2 的 0x201 收，`[8]` 按 0x202 收；按这次观察，它们对应物理右/左。两者都要看，避免按旧语义名字认错。此次先定位关闭链路，未改电调编号或输出方向。

## 烧录新排查固件后可直接双击

- `Open-Shoot.cmd`：直接打开第 5 组，看到摩擦请求、发射请求、实际转速、到速状态和输出。
- `Open-Shoot-Stop.cmd`：直接打开第 8 组，检查“关了以后为什么还转”。两者都不用输入编号。

文件夹：`E:/LEARN_CODE/OMNI_INFANTRY/OMNI_freeRTOS/OMNI_jpl/Docs/LinkScope`。可把这个文件夹路径粘贴到 Windows 文件资源管理器顶部地址栏，按回车，再双击文件。不是在 LinkScope 的变量表中输入编号。

第 8 组新增字段需要本次新固件和匹配的 ELF。`queued_output_raw` 是最近被 HAL 接受的原始输出，`output_queue_count` 是接受次数；计数继续增加且输出变为 0，只证明新的零值报文已装入邮箱，不代表电调必然收到。

## 该选哪一组

| 预置 | 用途 | 先观察什么 |
|---|---|---|
| `overview.ini` | 底盘能动，但上装不动 | `debug_ins.valid`、`debug_gimbal.faults`、`debug_shoot_task.faults` |
| `remote.ini` | 拨杆、摇杆、拨轮与动作是否对应 | 一次只动一个输入，看相应字段是否变化 |
| `imu.ini` | BMI088 初始化、静止标定 | 两个芯片 ID、初始化失败次数、静止样本数 |
| `gimbal.ini` | 归中、Yaw/Pitch 不响应 | IMU、两电机在线、CAN 就绪、最终输出 |
| `shoot.ini`（5） | 摩擦轮或拨弹不响应 | 两轮实际 RPM、到速状态、拨弹输出 |
| `shoot_stop.ini`（8） | 关闭后仍转 | 关闭命令、算法输出、HAL 接受的输出与实际 RPM |
| `motors.ini` | CAN2 上一整组设备无响应 | CAN2 收包计数，以及每个上装电机的在线状态 |
| `links.ini` | NUC 与裁判系统联调 | 收包计数是否增长、解析是否成功、数据是否新鲜 |

这些是“观察窗口”，不会写入控制变量。20 Hz 的窗口不能证明每个 1 ms 控制周期都正常；瞬时问题应结合故障计数或锁存信息分析，而不是因为曲线没看到尖峰就认定没出过问题。

## 最值得先看的 12 个变量

| 变量 | 正常时怎么理解 | 异常时先查 |
|---|---|---|
| `debug_cmd.enabled` | 1 表示总开关及遥控状态允许动作 | 拨杆位置、`debug_remote.online` 与帧龄 |
| `debug_cmd.neutral` | 1 表示相关输入已回中 | CH1～4、鼠标是否仍有偏量 |
| `debug_ins.initialized` | 1 表示 BMI088 寄存器初始化成功；还不等于姿态已标定 | 姿态标定是否完成继续看 `valid` 与静止样本数 |
| `debug_ins.valid` | 1 表示本次姿态可用 | SPI 采样、初始化失败或标定被打断 |
| `debug_bmi088.init_failures` | 稳定后不应持续增长 | `imu.ini` 中失败的寄存器与读回值 |
| `debug_gimbal.faults` | 0 表示云台没有记录当前阻断原因 | 对照源码的 `GIMBAL_BLOCK_*` 位标志 |
| `debug_gimbal.mode` | 0 放松、1 归中、2 手动稳定、3 自瞄、4 归中故障 | 一直为 0 看许可；4 检查方向、归中值与限位 |
| `debug_gimbal.yaw_online` | 1 表示 Yaw 反馈新鲜 | CAN1 / 0x207 的供电、ID、反馈帧 |
| `debug_gimbal.pitch_online` | 1 表示 Pitch 反馈新鲜 | CAN2 / 0x208 的供电、ID、反馈帧 |
| `debug_cmd.friction` | 1 只是“请求开摩擦轮” | 还要满足 shoot 层的其他许可 |
| `debug_shoot_task.faults` | 0 表示发射任务没有当前阻断原因 | 对照下表的发射位标志 |
| `debug_shoot_task.gimbal_ready` | 1 表示云台状态新鲜且归中完成 | 云台没归中时发射层继续保持禁止输出 |

`faults` 是位标志，可以同时包含多个原因。不要通过强制改 `permit`、`valid` 或 `centered` 来消掉问题；应沿着没有满足的条件查下去。

| 位值 | `debug_gimbal.faults` | `debug_shoot_task.faults` |
|---|---|---|
| 1 | 未使能或指令超时 | 未使能或指令超时 |
| 2 | 电机反馈不新鲜 | 发射电机反馈不新鲜 |
| 4 | CAN 未就绪 | CAN 未就绪 |
| 8 | IMU 不可用 | 控制周期异常 |
| 16 | 控制周期异常 | 云台未归中或状态过期 |
| 32 | 等待输入回中 | 没有请求开启摩擦轮 |
| 64 | 归中超时故障 | 尚未武装且扳机被按住，需要释放 |
| 128 | — | 等待两个摩擦轮到速 |
| 256 | — | 堵弹处理已锁存 |
| 512 | — | 电机模块拒绝输出提交 |

例如云台 `faults=10` 是 8+2，表示 IMU 和电机反馈两个条件都没满足；发射 `faults=48` 是 16+32，表示云台未准备好，同时摩擦轮命令没有打开。主动 disable 或关闭摩擦轮也会出现在此表中，不一定代表硬件损坏。

## 各层变量速查

| 字段 | 含义与单位 |
|---|---|
| `debug_cmd.ch1/ch2` | 右摇杆左右/上下，中心附近为 0，约 −660～660 |
| `debug_cmd.ch3/ch4` | 左摇杆左右/上下，中心附近为 0，约 −660～660 |
| `debug_cmd.sw1/sw2` | 工程变量名；上=1、中=3、下=2，不是按 1、2、3 排列 |
| `debug_cmd.wheel` | 背面拨轮，中心附近为 0；先观察再确认推哪边是正值 |
| `debug_cmd.spin/friction/fire/auto_aim` | 解释输入后得到的动作请求，0=否、1=是 |
| `debug_bmi088.accel_id/gyro_id` | 预期分别为 0x1E=30、0x0F=15；全 0/255 先查 SPI/片选 |
| `debug_bmi088.init_attempts/init_failures` | 初始化尝试/失败累计次数，比较增量 |
| `debug_bmi088.reg/actual/expected/mask` | 校验寄存器、读回值、期望值、比较掩码；比较的是 `actual & mask` 与 `expected & mask` |
| `debug_ins.stationary_samples` | 本次连续静止样本数；需要约 500 个有效样本才能完成启动标定 |
| `debug_ins.accel_norm` | 加速度模长，静止约 9.81 m/s² |
| `debug_ins.gyro_norm` | 角速度模长，rad/s；标定要求小于约 0.08 rad/s |
| `debug_ins.sample_errors/generation` | 采样失败累计次数/姿态估计重新初始化代次 |
| `debug_ins.yaw_deg/pitch_deg` | 姿态角，度；不等于电机编码器原始格数 |
| `debug_gimbal.command_age_ms` | 最后指令的数据年龄，ms；不是任务从 Queue 取出后过了多久 |
| `debug_gimbal.tx_ready` | 云台所用 CAN 通道和发送状态是否允许控制 |
| `debug_gimbal.yaw_output/pitch_output` | 云台提交给电机层的原始输出码，不是安培；非零仍需核对总线是否真正发出 |
| `debug_gimbal.yaw_relative_deg/pitch_relative_deg` | 电机编码器相对于机械归中值的角度，度 |
| `debug_shoot_task.motors_ready/can_ready` | 三个发射电机反馈是否新鲜/发送通道是否就绪 |
| `debug_shoot.armed` | 发射控制已接受启动条件；持续扣住扳机时可能等待释放 |
| `debug_shoot.friction_ready` | 两个摩擦轮达到目标转速并保持一段时间 |
| `debug_shoot.jam_latched` | 堵弹处理后的锁存状态，需按设计释放扳机 |
| `debug_shoot.raw[0]/[1]/[2]` | 左摩擦轮/右摩擦轮/拨弹电机的原始输出码 |
| `debug_motor[i].online/age_ms/frames` | 第 i 个电机是否在线/反馈年龄/收到的有效反馈总数 |
| `debug_motor_can.started[0]/[1]` | CAN1/CAN2 是否启动；启动成功不等于收到电机反馈 |
| `debug_motor_can.rx_frames/errors/queue_drops` | 两条总线各自的收帧/错误/接收队列溢出累计次数，数组 0=CAN1、1=CAN2 |
| `debug_transmission.online` | NUC 链路是否在 500 ms 内收到有效角度包或心跳 |
| `debug_transmission.aim_valid/age_ms` | 角度包是否在 100 ms 内有效/角度包的年龄；4294967295 表示没有保留的有效目标，心跳不能让旧角度变新 |
| `debug_transmission.target_frames/checksum_errors/tx_frames` | NUC 合法角度包/校验失败/已接受发送的帧计数 |
| `debug_nuc_usb.configured/rx_packets/tx_packets/dropped_packets` | USB 是否已枚举为 CDC/收到包数/发出包数/丢包数；USB 已枚举不等于 NUC 已发送角度 |
| `debug_transmission.raw_target_flag` | 旧 NUC 协议的原始标志；没有定义就不推断为“已识别目标” |
| `debug_referee.online` | 裁判链路在 1000 ms 内收到 CRC 正确的帧，不等于每种业务数据都新鲜 |
| `debug_referee.status_fresh/power_heat_fresh` | 0x0201 状态在 1500 ms 内有效/0x0202 功率热量在 500 ms 内有效 |
| `debug_referee.valid_frames/crc_errors/header_errors/invalid_payloads` | 有效帧/帧 CRC16 错误/帧头 CRC8 错误/业务长度或数值错误累计次数 |
| `debug_referee.robot_id/chassis_power_limit_w/heat_17mm_1/heat_limit` | 机器人 ID、底盘功率上限 W、枪口热量、热量上限；与对应 fresh 标志一起看 |

电机下标对应语义 ID：0=FL、1=FR、2=RL、3=RR、4=Yaw、5=Pitch、6=拨弹、7=左摩擦、8=右摩擦。注意 `debug_motor[]` 的顺序与 `debug_shoot.raw[]` 的顺序不同。

## 本次“不归中、摩擦轮不转”的已知软件链路

旧代码向陀螺仪 `GYRO_BANDWIDTH`（0x10）写 0x01，读回后要求整个字节都等于 0x01。但 BMI088 的该寄存器 bit7 是只读的 1，实际正常读回为 0x81；整个字节比较会错误地判初始化失败。正确处理是只比较配置位，保留真正的通信与写入校验。[Bosch 数据手册 §5.5.6，第 39 页](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)

因此可能出现：底盘正常 → BMI088 初始化一直失败 → INS 不可用 → 云台无法归中 → 发射任务等不到云台归中完成。修复后先看失败计数停止增长，再静止等待标定完成；这条软件原因已经定位，但引脚、CAN 接线、电机方向、归中编码器值和实车运动效果仍需上板验证。

旧七组已有离线符号检查记录。OMNI9_26 增加第 8 组后重新检查所有配置和 ELF 字段；这仅验证变量可解析，没有代替用户连接硬件或确认实际曲线。

## 2026-09-27：闭环停车与底盘跟随

右拨杆现在是：上=disable，中=跟随云台，下=小陀螺。中位只在云台已归中、参考有效且快照新鲜时产生跟随角速度；否则跟随输出为0，平移仍保持原来的车体坐标控制。当前没有增加云台坐标系下的平移变换。

`debug_cmd.follow` 表示请求跟随；`debug_follow.active` 表示实际满足跟随条件；`debug_follow.error_deg` 是车头与云台的角差，`debug_follow.wz_rad_s` 是跟随环要求的车体角速度。参数在 `Task/chassis/chassis_config.h` 的 `CHASSIS_FOLLOW_*`，默认限速1.5 rad/s、死区2度，最终仍经过底盘加速度和功率限制。先悬空/低速确认误差变小；若越追越远，disable后检查方向，不能靠加大PID补救。

左拨杆上/G键关闭摩擦轮时，运行过且许可仍有效的摩擦轮进入零转速闭环制动。独立停车PID默认 Kp=1.5，Ki=Kd=0，输出限幅2500；两轮均低于100 rpm持续50 ms后清零，最长1.5 s后也退出。制动中拨弹始终为0；再次开启必须重新释放扳机、重新到速。参数在 `Task/shoot/shoot_config.h`。这些是首轮调试值，实际停车时间、发热和母线回馈需实测。

右上disable、遥控失联、反馈失效或控制超时立即清零，不继续电制动。普通关闭时 `friction=0` 但 `raw` 短暂非零现在是预期现象：看 `braking=1`，且输出方向应与实测转速相反。`brake_timeouts` 增长表示在期限内未确认停稳，不会无限制施加制动力。

物理左摩擦已按用户确认改为 CAN2 ID2（反馈0x202），物理右为 ID1（反馈0x201）；`debug_motor[7]` 现在对应左，`[8]` 对应右。目标符号同步调整为左−6200、右+6200 rpm，保留原先每个CAN编号的驱动方向。拨弹仍为 CAN2 ID3/0x203，未猜改编号。

### 拨弹不动的最短排查

使用新固件和匹配ELF打开 `Open-Shoot.cmd`：

1. `offline_mask` 必须为0。1=左摩擦离线，2=右摩擦离线，4=拨弹离线，可相加。7表示三台均离线。原来的 `faults=2` 仅是总提示。
2. 两轮实际转速要跟上各自 `target_rpm`；`friction_ready=1` 才可拨弹。只知道轮子在转不足以证明到速。方向错误、转速不足或波动大都要先处理，不用删除门控。
3. `armed=1`、到速后松开再拨动，`debug_cmd.fire` 应从0变1。单发不会保存提前扣住的一次请求。
4. `raw[2]` 非零而 `debug_motor[6].speed_rpm` 仍为0，继续查拨弹的 CAN2/0x203、C610编号、减速器/拨盘阻力和电流；不能仅凭这两项就认定电机损坏。

本次软件测试能证明闭环停车和门控/输出路径符合设计，不能替代拨弹实际动作测试。未收到当前实车的到速、反馈和输出值前，不声称拨弹故障已排除。
