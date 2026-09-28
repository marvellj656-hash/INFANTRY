# NUC 通信排查（OMNI9_26）

链路：主控 transmission 任务 → USB CDC → NUC 串口读取/BCP解析程序 → UDP → PlotJuggler。
照片中选的是 UDP Server。曲线空白不能证明 USB 断线；变量列表也可能来自之前的数据。

## 先看主控，不急着改协议
原 Open-LinkScope.cmd 第14组查看USB，第15组查看姿态发送；新增第18组只勾选三条曲线：
`debug_nuc_usb.tx_packets`（提交次数）、`debug_nuc_usb.tx_completed`（USB完成次数）、
`debug_transmission.tx_frames`（提交的姿态帧数）。新变量需要烧录匹配的新ELF；未烧录时先用14/15组旧变量。

| 观察 | 下一步 |
|---|---|
| initialized=0 | 查任务是否启动、init_failures、USB初始化 |
| initialized=1、configured=0 | 查数据线、NUC设备枚举、USB时钟/接线，尚不到协议层 |
| configured=1、tx_packets只增加一次、tx_busy持续增加 | 查NUC是否打开并读取串口、USB端点/中断；提交不等于完成 |
| tx_completed增长、tx_frames=0 | 查tx_reference_valid、云台归中、IMU有效状态；当前无有效归中参考只发心跳 |
| tx_frames和tx_completed均持续增长 | 主控正在提交姿态且USB有完成；去NUC看收到的原始字节、解析错误及UDP转发 |
| rx_packets=0 | 只表示没有收到NUC回包，不能证明主控没有发送 |

`debug_transmission.online/heartbeat_online` 是主控观察到的NUC输入状态，不是NUC接收成功确认。
完成回调也不证明NUC应用完成了协议解析。

## NUC终端先查设备和占用
执行 `ls -l /dev/serial/by-id/`，确认是否存在主控CDC设备及其指向的ttyACM编号。
执行 `dmesg | tail -n 50` 查看插拔枚举/反复断连；部分系统需要sudo查看日志。
对实际设备执行 `fuser -v /dev/ttyACM0`，把ttyACM0换成上一步的实际编号。
检查NUC程序的设备路径和权限，避免重插后编号变化。不要让两个读串口的程序同时抢数据。
NUC解析日志应分别记录收到字节数、有效姿态帧、校验错误、未知ID、UDP发送数。
如果有效姿态帧增长而PlotJuggler没变化，再查UDP目的地址/端口、PlotJuggler监听端口和解析插件，并把变量拖到曲线区。

## 当前BCP报文（下标从0开始）
姿态31字节：`FF 01 20 19 DATA[25] SC AC`。
帧偏移5/9：yaw/pitch；17：队伍；21/25：yaw/pitch角速度。
这些数值均是有符号int32小端整数，角度/角速度分别为度、度每秒，乘1000后传送；不是float字节。
偏移4保留为0，偏移13的原开火字段保持0。
心跳7字节：`FF 01 F0 01 sequence SC AC`，100ms周期；姿态目标周期10ms。
SC先把4字节头一起求和，AC先取这个和；再逐个DATA字节更新SC、AC，均截断为8位。不是CRC。
NUC必须缓存字节流：找到FF → 收齐4字节头 → 按LEN+6收齐 → 验证SC/AC → 按ID分发。
一次read可能只有半帧，也可能有多帧；未知ID按帧长度跳过，不能固定每次切31字节。

旧RTT角度字段和校验相同，但新版加了短心跳及原保留位的角速度。
若NUC旧程序固定只支持31字节，可临时在 Modules/transmission/transmission_tx.h 把
TRANSMISSION_TX_HEARTBEAT_ENABLE 从1改0，重新编译烧录进行A/B测试；只在云台归中后比较姿态接收。
这只是定位兼容性，不能代替修好NUC的变长包解析。尚未取得NUC源码，当前不擅自改默认协议。

另外当前手动模式会不断重置NUC的pitch参考，因此tx_pitch_deg可长期为0；这不等于没发包。
