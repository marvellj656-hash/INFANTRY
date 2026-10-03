#ifndef OMNI_JPL_GIMBAL_CONFIG_H
#define OMNI_JPL_GIMBAL_CONFIG_H

/* 本文件只管云台：机械零点、限位、归中，以及两轴双环PID。
 * 底盘跟随在 chassis/chassis_config.h；遥控倍率在 cmd/cmd_config.h。
 * 电机CAN与电调编号以 Task/robot.c 的 motor_config 为准。
 * PID命名：YAW/PITCH=轴，A/V=角度环/速度环，IMU/AUTO=手动/自瞄。
 * 角度环：度 -> rad/s；速度环：rad/s -> 目标转矩电流(A)。
 * *_INTEGRAL_* 限制积分输出，*_MAX_* 限制总输出，单位均与该环输出一致。
 * 每组Kp/Ki/Kd/积分限幅/总限幅都由gimbal_control.c读取；Ki非零还需积分限幅非零。
 * 速度环换了输出单位，不能照搬原电压码增益；以下电流参数是待实车调试的起点。
 */

/*云台编码器归中*/
/* 可选侧装配置只切换机械编码器中心；不会额外旋转坐标系。 */
// #define GIMBAL_SIDEWAYS
#ifdef GIMBAL_SIDEWAYS
/* 云台yaw轴编码器归中值(侧身) */
#define CENTER_ECD_YAW 3818
#else
/* 云台yaw轴编码器归中值 */
#define CENTER_ECD_YAW 5800
#endif

/* 云台pitch轴编码器归中值 */
#define CENTER_ECD_PITCH 3532
/* pitch轴最大仰角 */
#define PIT_ANGLE_MAX 30.0f
/* pitch轴最大俯角 */
#define PIT_ANGLE_MIN -20.0f

/* 云台控制周期 (ms) */
#define GIMBAL_PERIOD 1
/* 云台回中初始化时间 (ms) */
#define BACK_CENTER_TIME 100

/* 两轴电流指令上限1.5A（不是持续堵转许可），编码为8192。
 * 手册：±16384对应±3A。PID与电机层使用同一个限流配置。
 * 先调速度环，再调角度环；速度环D从0开始，避免放大陀螺噪声。 */
#define GIMBAL_CURRENT_LIMIT_A 1.5f
#define GIMBAL_CURRENT_RAW_PER_A (16384.0f / 3.0f)
#define GIMBAL_CURRENT_LIMIT_RAW ((int16_t)(GIMBAL_CURRENT_LIMIT_A * GIMBAL_CURRENT_RAW_PER_A))
_Static_assert(GIMBAL_CURRENT_LIMIT_A > 0.0f && GIMBAL_CURRENT_LIMIT_A <= 3.0f,
               "GM6020 current command must be within 0..3 A");

/* -------------------------------- 云台电机PID参数 ------------------------------- */
/* 云台yaw轴电机PID参数 */
/* imu速度环 */
#define YAW_KP_V_IMU 0.4f
#define YAW_KI_V_IMU 0.0f
#define YAW_KD_V_IMU 0.0f
#define YAW_INTEGRAL_V_IMU 0.0f
#define YAW_MAX_V_IMU GIMBAL_CURRENT_LIMIT_A
/* imu角度环 */
#define YAW_KP_A_IMU 0.14f
#define YAW_KI_A_IMU 0
#define YAW_KD_A_IMU 0.002f
#define YAW_INTEGRAL_A_IMU 0
#define YAW_MAX_A_IMU 25
/* auto速度环 */
#define YAW_KP_V_AUTO 0.4f
#define YAW_KI_V_AUTO 0.0f
#define YAW_KD_V_AUTO 0.0f
#define YAW_INTEGRAL_V_AUTO 0.0f
#define YAW_MAX_V_AUTO GIMBAL_CURRENT_LIMIT_A
/* auto角度环 */

#define YAW_KP_A_AUTO 0.25f
#define YAW_KI_A_AUTO 0
#define YAW_KD_A_AUTO 0.008f
#define YAW_INTEGRAL_A_AUTO 0
#define YAW_MAX_A_AUTO 25
/* 云台PITCH轴电机PID参数 */
/* imu速度环 */
#define PITCH_KP_V_IMU 0.4f
#define PITCH_KI_V_IMU 0.2f
#define PITCH_KD_V_IMU 0.0f
#define PITCH_INTEGRAL_V_IMU 0.3f
#define PITCH_MAX_V_IMU GIMBAL_CURRENT_LIMIT_A

/* imu角度环 */
#define PITCH_KP_A_IMU 0.65f
#define PITCH_KI_A_IMU 0.0f
#define PITCH_KD_A_IMU 0.00015f
#define PITCH_INTEGRAL_A_IMU 0.0f
#define PITCH_MAX_A_IMU 20

/* auto速度环 */
#define PITCH_KP_V_AUTO 0.4f
#define PITCH_KI_V_AUTO 0.2f
#define PITCH_KD_V_AUTO 0.0f
#define PITCH_INTEGRAL_V_AUTO 0.3f
#define PITCH_MAX_V_AUTO GIMBAL_CURRENT_LIMIT_A
/* auto角度环 */
#define PITCH_KP_A_AUTO 0.25f
#define PITCH_KI_A_AUTO 0.0f
#define PITCH_KD_A_AUTO 0.001f
#define PITCH_INTEGRAL_A_AUTO 0.0f
#define PITCH_MAX_A_AUTO 20

/* 归中必须在限定时间完成，否则等待重新关闭总开关。 */
#define GIMBAL_HOME_TIMEOUT_S 2.0f
#define GIMBAL_MAX_RATE_DEG_S 180.0f

#endif /* OMNI_JPL_GIMBAL_CONFIG_H */
