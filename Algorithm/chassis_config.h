#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H
/* 首次上车参数：单位统一，禁止把 RPM 当 rad/s 或把电流码值当安培。 */
#define CHASSIS_MAX_SPEED_MM_S 300.0f
#define CHASSIS_MAX_YAW_RAD_S 0.5f
#define CHASSIS_ACCEL_MM_S2 500.0f
#define CHASSIS_YAW_ACCEL_RAD_S2 1.0f
/* 几何及轮序沿用旧工程 omni_calc：电机 0..3 对应 0x201..0x204。 */
#define CHASSIS_WHEEL_PERIMETER_MM 471.0f
#define CHASSIS_GEAR_RATIO 19.0f
#define CHASSIS_RADIUS_MM 230.0f
#define CHASSIS_MAX_MOTOR_RPM 1000.0f
/* C620 的控制码值：16384 对应 20 A。这里 2000 约为 2.44 A，不是 2000 mA。 */
#define CHASSIS_CURRENT_LIMIT_RAW 2000
#define CHASSIS_KP 6.0f
#define CHASSIS_KI 0.2f
#define CHASSIS_KD 0.0001f
#define CHASSIS_I_LIMIT 500.0f
/* 无裁判数据阶段的估算预算，绝非实测功率保证；禁止据此宣称比赛合规。 */
#define CHASSIS_BRINGUP_POWER_W 30.0f
/* 暂用物理模型参数，电阻/空载损耗需用实测输入功率标定。 */
#define POWER_RAW_TO_A (20.0f / 16384.0f)
#define POWER_ROTOR_KT (0.3f * 0.05207463310219f)
#define POWER_EQUIVALENT_R 0.3f
#define POWER_IDLE_W 5.0f
#endif
