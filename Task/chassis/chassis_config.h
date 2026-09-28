#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H
/* 底盘参数：单位统一，禁止把 RPM 当 rad/s 或把电流码值当安培。 */
#define CHASSIS_MAX_SPEED_MM_S 6000.0f /* 沿用原普通速度上限，mm/s。 */
#define CHASSIS_MAX_YAW_RAD_S 7.0f
#define CHASSIS_ACCEL_MM_S2 4000.0f
/* SW1下位小陀螺的目标角速度；仍受总限速、斜坡、轮速和功率限制。 */
#define CHASSIS_SPIN_RAD_S 4.5f
/* 跟随角度环：SW2中位/下位且未开小陀螺时，让云台相对车头的角差趋近0。
 * 本组CHASSIS_FOLLOW_*才是跟随PID；后面的CHASSIS_KP/KI/KD是四轮速度环。
 * 先用小角度确认底盘转动让角差缩小，再提高速度。
 * 与旧 RTT 符号一致：新的 relative_deg 已取过编码器负号，所以这里为+1。 */
#define CHASSIS_FOLLOW_DIRECTION 1.0f
#define CHASSIS_FOLLOW_KP 0.27f
#define CHASSIS_FOLLOW_KI 0.0f
/* 积分项也是角速度：只增大 Ki、但此处仍为 0，积分就仍然被夹在 0。 */
#define CHASSIS_FOLLOW_I_LIMIT_RAD_S 0.0f
#define CHASSIS_FOLLOW_KD 0.001f
#define CHASSIS_FOLLOW_DEADBAND_DEG 0.0f
#define CHASSIS_FOLLOW_MAX_RAD_S 20.0f
/* 任务和主机测试共用这一份配置，避免任务里藏着另一个积分限幅。 */
#define CHASSIS_FOLLOW_PID_CONFIG \
    INIT_PID_CONFIG(CHASSIS_FOLLOW_KP, CHASSIS_FOLLOW_KI, CHASSIS_FOLLOW_KD, \
                    CHASSIS_FOLLOW_I_LIMIT_RAD_S, CHASSIS_FOLLOW_MAX_RAD_S, \
                    PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Trapezoid_Intergral | PID_RttControl)
/* 几何及轮序沿用旧工程 omni_calc：电机 0..3 对应 0x201..0x204。 */
#define CHASSIS_WHEEL_PERIMETER_MM 471.0f
#define CHASSIS_GEAR_RATIO 19.0f
#define CHASSIS_RADIUS_MM 230.0f
#define CHASSIS_MAX_MOTOR_RPM 9000.0f
/* C620 的控制码值：16384 对应 20 A。普通模式沿用原 8000 码值，约 9.77 A，不是 8000 mA。 */
#define CHASSIS_CURRENT_LIMIT_RAW 8000
/* 四轮速度环：转子RPM -> C620电流码。与上面的跟随角度环是两层控制。 */
#define CHASSIS_KP 6.0f
#define CHASSIS_KI 0.2f
#define CHASSIS_KD 0.0001f
#define CHASSIS_I_LIMIT 500.0f
/* 默认优化选项沿用原底盘；其他滤波/变速积分按需要开启并配置参数。 */
#define CHASSIS_PID_IMPROVE                                                                        \
    (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement)
/* 每行对应 CAN1 的 0x201 / 0x202 / 0x203 / 0x204。
 * INIT_PID_CONFIG 参数：Kp、Ki、Kd、积分项限幅、电流输出限幅、优化开关。
 * 初值保持原来的统一参数。需要单轮调参时，只修改该行的对应数值。
 * 改完重新编译烧录，不在车运行时通过 LinkScope 改控制参数。 */
#define CHASSIS_SPEED_PID_CONFIGS                                                                  \
    {INIT_PID_CONFIG(CHASSIS_KP, CHASSIS_KI, CHASSIS_KD, CHASSIS_I_LIMIT,                          \
                     CHASSIS_CURRENT_LIMIT_RAW, CHASSIS_PID_IMPROVE),                              \
     INIT_PID_CONFIG(CHASSIS_KP, CHASSIS_KI, CHASSIS_KD, CHASSIS_I_LIMIT,                          \
                     CHASSIS_CURRENT_LIMIT_RAW, CHASSIS_PID_IMPROVE),                              \
     INIT_PID_CONFIG(CHASSIS_KP, CHASSIS_KI, CHASSIS_KD, CHASSIS_I_LIMIT,                          \
                     CHASSIS_CURRENT_LIMIT_RAW, CHASSIS_PID_IMPROVE),                              \
     INIT_PID_CONFIG(CHASSIS_KP, CHASSIS_KI, CHASSIS_KD, CHASSIS_I_LIMIT,                          \
                     CHASSIS_CURRENT_LIMIT_RAW, CHASSIS_PID_IMPROVE)}
/* 无裁判数据阶段的估算预算，绝非实测功率保证；禁止据此宣称比赛合规。 */
#define CHASSIS_BRINGUP_POWER_W 60.0f
/* 暂用物理模型参数，电阻/空载损耗需用实测输入功率标定。 */
#define POWER_RAW_TO_A (20.0f / 16384.0f)
#define POWER_ROTOR_KT (0.3f * 0.05207463310219f)
#define POWER_EQUIVALENT_R 0.3f
#define POWER_IDLE_W 5.0f
#endif
