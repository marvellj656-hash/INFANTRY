# 主机回归测试

工程根目录运行：

```powershell
./Tests/run_native_tests.ps1
```

默认编译器 E:/mingw64/bin/gcc.exe，可用 -GccPath 覆盖。
输出位于系统临时目录 omni-native-tests。测试不访问硬件，不烧录。
共 15 组：cmd、referee、referee_bsp、referee_ui、transmission、nuc_usb、aux_control、imu、pid、chassis、can、remote、protocol、modules、attitude。
完整覆盖范围与局限见 [迁移验证记录](../Docs/migration_status.md)。

EKF 测试使用工程自带 CMSIS-DSP 的六个矩阵函数，算法与 ARM 目标相同；
__GNUC_PYTHON__ 仅让 CMSIS 使用主机兼容类型定义。
CAN/UART/SPI 测试用假硬件，不代表真实时序、接线和电机方向已验证。
