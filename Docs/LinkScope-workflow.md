> 2026-09-26：接口已更新。当前接口与验证范围以 [架构指南](architecture.md) 为准。

# LinkScope 固定使用流程

本机 LinkScope 自带 GDB 7.6.1 对当前 ELF 的变量查询卡住；同一个 ELF 用 ARM GCC 随带的 GDB 10.2 正常解析。已替换 LinkScope/gdb/gdb.exe，旧版留在 gdb-7.6.1.backup.exe。无需每次编译替换 GDB。

1. CLion 编译 Debug 成功。
2. 烧录本次 build-arm-debug/OMNI_jpl.elf 对应固件。
3. 停止 LinkScope 采样，重新选择这同一个 ELF，刷新符号缓存。LinkScope 会复制符号到自己的临时文件，重新编译不等于自动刷新。
4. 导入 Docs/LinkScope-diagnose.ini，开始观察。保存自己的配置；新工程变量为 debug_remote.ch1/ch2，不是旧 RTT 的 rc_now。

若变量窗口再次卡住：关闭 LinkScope，确认它的 gdb 子进程退出后重开；不要结束 CLion 的调试器。可用命令行离线检查（不连接主控）：

    "E:/FOQQ/Linkscope/LinkScope-x86-V1.3.0/gdb/gdb.exe" -q -nx -batch -ex "file E:/LEARN_CODE/OMNI_INFANTRY/OMNI_freeRTOS/OMNI_jpl/build-arm-debug/OMNI_jpl.elf" -ex "ptype debug_remote"

能显示类型表示 ELF 存在该变量；不能据此证明板上固件与 ELF 一致。仅普通局部变量、优化删除的变量不适合直接实时监视；调试状态使用已有全局 debug 结构。

## 右摇杆不响应

先将 SW2 放中位、CH1/2/3 归中，再推右摇杆。比较操作前后计数的增量，启动时非零不代表持续故障。

- CH1/CH2 不跟随：检查 remote.age_ms、invalid_frames、uart_errors。
- CH1/CH2 跟随，但 fault_flags 非零：1=遥控未使能/超时，2=电机反馈缺失或超过20ms，4=控制周期异常，8=CAN未就绪/发送失败，16=等待CH1/2/3归中。多种故障按位相加。
- last_fault_flags 保留最近真实故障；四个故障计数可捕获瞬时故障。其中 timing_faults 包含首次执行 elapsed=0，观察后续是否增长。
- fault_flags=0 且 armed=1：继续比较 target、velocity、四轮 target_rpm/measured_rpm/current_raw 和 power_scale，才进入速度环或功率限制诊断。

当前只增加观测，没有放宽保护、调整 PID 或确认硬件故障。此配置新增诊断字段需要烧录本次编译固件后使用。

## 2026-09-20 补充：已复现并固定的原因

上一轮只替换 GDB 不够。相同 ELF 的交互式 `info variables`，含 -g3 宏信息时约23秒，去除 .debug_macro/.debug_macinfo 后约25毫秒。现已在根 CMakeLists.txt 添加 Debug 的 -g2 / -gdwarf-4，覆盖生成工具链的 -g3；全量重编译后约28毫秒，仍保留变量、类型与源码行号。修改前后可烧录二进制 SHA256 完全相同，只有调试信息变化。

LinkScope 强制关闭后还可能遗留 OpenOCD 占用3333端口。以后双击：

    E:/FOQQ/Linkscope/LinkScope-x86-V1.3.0/Start-LinkScope.cmd

脚本只在本安装没有运行中的 LinkScope 窗口时清理本安装的 GDB/OpenOCD 和两个临时符号副本，不会关闭 CLion 调试器，不会复位或烧录主控。若另一套调试器正在使用 ST-Link，应先正常结束该调试会话。

本次实机只读验证：板上 Flash 与当前构建一致；控制周期计数持续增长，SWD 内存可读。遥控 online=1、age=11ms，SW2=1（停止档）；底盘 motor[1]（CAN1/0x202）frames=0，其余三轮反馈持续增长，当前fault=3。因此此刻停机不能归因于PID，需检查0x202对应电调的ID/接线/供电，并把SW2设中位后归中使能。实时采样数值随硬件状态变化，此记录只代表检查时刻。
