param([string]$Project = (Split-Path -Parent $PSScriptRoot),
      [string]$GccPath = "E:/mingw64/bin/gcc.exe",
      [string]$SdkProject = $Project)
$ErrorActionPreference = 'Stop'
$gcc = $GccPath
$output = Join-Path ([System.IO.Path]::GetTempPath()) "omni-native-tests"
New-Item -ItemType Directory -Force -Path $output | Out-Null
$includes = @('Task','Task/chassis','Modules','Algorithm','Bsp') | ForEach-Object { "-I$Project/$_" }
$includes += Get-ChildItem "$Project/Algorithm","$Project/Bsp" -Filter *.h -Recurse |
    Select-Object -ExpandProperty DirectoryName -Unique | ForEach-Object { "-I$_" }
$algorithm = @('Task/chassis/chassis_control.c','Task/cmd/cmd_mapping.c','Algorithm/pid/pid.c','Algorithm/ramp/ramp.c','Algorithm/omni_kinematics/omni_kinematics.c','Algorithm/power_model/power_model.c')
$suites = @(
    @{ Name="brake_follow"; Mock=""; Files=@("Tests/test_brake_follow.c","Task/shoot/shoot_control.c","Algorithm/chassis_follow/chassis_follow.c","Algorithm/pid/pid.c","Algorithm/ramp/ramp.c") },
    @{ Name='shoot_path'; Mock='Tests/can_support'; Files=@('Tests/test_shoot_path.c','Task/cmd/cmd_mapping.c','Task/shoot/shoot_control.c','Modules/motor/dji_motor.c','Modules/motor/dji_protocol.c','Algorithm/pid/pid.c','Algorithm/ramp/ramp.c') },
    @{ Name='cmd'; Mock=''; Files=@('Tests/test_cmd.c') + $algorithm },
    @{ Name='referee'; Mock='Tests/support'; Files=@('Tests/test_referee.c','Modules/referee/referee_protocol.c','Modules/referee/referee.c') },
    @{ Name='referee_bsp'; Mock='Tests/referee_bsp_support'; Files=@('Tests/test_referee_bsp.c','Bsp/referee/bsp_referee.c','Bsp/uart/bsp_uart.c') },
    @{ Name='referee_ui'; Mock=''; Files=@('Tests/test_referee_ui.c','Modules/referee_ui/referee_ui.c') },
    @{ Name='transmission'; Mock='Tests/can_support'; Files=@('Tests/test_transmission.c','Modules/transmission/transmission.c','Modules/transmission/transmission_protocol.c') },
    @{ Name='nuc_usb'; Mock='Tests/nuc_support'; ExtraMock='Tests/can_support'; Files=@('Tests/test_nuc_usb.c','Bsp/nuc/bsp_nuc.c') },
    @{ Name='aux_control'; Mock=''; Files=@('Tests/test_aux_control.c','Task/gimbal/gimbal_control.c','Task/shoot/shoot_control.c','Algorithm/pid/pid.c','Algorithm/ramp/ramp.c') },
    @{ Name='imu'; Mock='Tests/imu_support'; Files=@('Tests/test_imu.c','Modules/imu/bmi088.c','Bsp/imu/bsp_imu.c') },
    @{ Name='pid'; Mock=''; Files=@('Tests/test_pid.c','Algorithm/pid/pid.c') },
    @{ Name='chassis'; Mock=''; Files=@('Tests/test_chassis.c','Modules/motor/dji_protocol.c') + $algorithm },
    @{ Name='can'; Mock='Tests/can_support'; Files=@('Tests/test_motor_can.c','Bsp/can/bsp_can.c') },
    @{ Name='remote'; Mock='Tests/support'; Files=@('Tests/test_remote.c','Modules/remote/dbus_protocol.c','Bsp/uart/bsp_uart.c') },
    @{ Name='protocol'; Mock=''; Files=@('Tests/test_dji_feedback.c','Modules/motor/dji_protocol.c') },
    @{ Name='modules'; Mock='Tests/can_support'; Files=@('Tests/test_modules.c','Modules/remote/remote.c','Modules/remote/dbus_protocol.c','Modules/motor/dji_motor.c','Modules/motor/dji_protocol.c') }
)
foreach ($suite in $suites) {
    $options = @('-DCHASSIS_HOST_TEST','-std=c11','-Wall','-Wextra','-Werror')
    if ($suite.Mock) { $options += "-I$Project/$($suite.Mock)" }
    if ($suite.ExtraMock) { $options += "-I$Project/$($suite.ExtraMock)" }
    $options += $includes
    $options += $suite.Files | ForEach-Object { "$Project/$_" }
    $options += @('-lm','-o',"$output/$($suite.Name).exe")
    & $gcc @options
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $($suite.Name)" }
    & "$output/$($suite.Name).exe"
    if ($LASTEXITCODE -ne 0) { throw "Test failed: $($suite.Name)" }
}

# 与 ARM 固件编译同一份 EKF 和 CMSIS 矩阵算法，主机只替换平台头文件模式。
$matrix = @('init','add','sub','mult','trans','inverse') | ForEach-Object {
    "$SdkProject/Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_${_}_f32.c"
}
$ekf = @('Tests/test_attitude.c','Algorithm/QuaternionEKF/attitude.c',
    'Algorithm/QuaternionEKF/QuaternionEKF.c','Algorithm/kalman_filter/kalman_filter.c') | ForEach-Object { "$Project/$_" }
& $gcc '-D__GNUC_PYTHON__' '-std=c11' '-Wall' '-Wextra' '-Werror' @includes "-I$SdkProject/Drivers/CMSIS/DSP/Include" @ekf @matrix '-lm' '-o' "$output/attitude.exe"
if ($LASTEXITCODE -ne 0) { throw 'Compilation failed: attitude' }
& "$output/attitude.exe"
if ($LASTEXITCODE -ne 0) { throw 'Test failed: attitude' }
