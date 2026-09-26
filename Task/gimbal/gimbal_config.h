//
// Created by Administrator on 2026/9/25.
//

#ifndef OMNI_JPL_GIMBAL_CONFIG_H
#define OMNI_JPL_GIMBAL_CONFIG_H

// gimbal config of 2 yaw and pitch
// can rx_id controller
#define YAW_MOTOR_ID 0x207
#define PITCH_MOTOR_ID 0x208

/*云台编码器归中*/
// #define GIMBAL_SIDEWAYS
#ifdef GIMBAL_SIDEWAYS
#define SIDEWAYS_ANGLE 36
/* 云台yaw轴编码器归中值(侧身) */
#define CENTER_ECD_YAW 3818
#else
/* 云台yaw轴编码器归中值 */
#define CENTER_ECD_YAW 5800
#define SIDEWAYS_ANGLE 0
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

/* -------------------------------- 云台电机PID参数 ------------------------------- */
/* 云台yaw轴电机PID参数 */
/* imu速度环 */
#define YAW_KP_V_IMU 5000
#define YAW_KI_V_IMU 0
#define YAW_KD_V_IMU 0.5
#define YAW_INTEGRAL_V_IMU 0
#define YAW_MAX_V_IMU 30000
/* imu角度环 */
#define YAW_KP_A_IMU 0.14f
#define YAW_KI_A_IMU 0
#define YAW_KD_A_IMU 0.002f
#define YAW_INTEGRAL_A_IMU 0
#define YAW_MAX_A_IMU 25
/* auto速度环 */
#define YAW_KP_V_AUTO 7777 // 7000//6200// 5800// 4000
#define YAW_KI_V_AUTO 0
#define YAW_KD_V_AUTO 0.01
#define YAW_INTEGRAL_V_AUTO 0
#define YAW_MAX_V_AUTO 30000
/* auto角度环 */

#define YAW_KP_A_AUTO 0.25f // 0.15f  //0.10f
#define YAW_KI_A_AUTO 0
#define YAW_KD_A_AUTO 0.008f
#define YAW_INTEGRAL_A_AUTO 0
#define YAW_MAX_A_AUTO 25
/* 云台PITCH轴电机PID参数 */
/* imu速度环 */
#define PITCH_KP_V_IMU 4500 // 5000//6000
#define PITCH_KI_V_IMU 4000
#define PITCH_KD_V_IMU 0.003
#define PITCH_INTEGRAL_V_IMU 1500
#define PITCH_MAX_V_IMU 30000

/* imu角度环 */
#define PITCH_KP_A_IMU 0.65f
#define PITCH_KI_A_IMU 0.0f
#define PITCH_KD_A_IMU 0.00015f
#define PITCH_INTEGRAL_A_IMU 0.0f
#define PITCH_MAX_A_IMU 20

/* auto速度环 */
#define PITCH_KP_V_AUTO 4500 // 4000
#define PITCH_KI_V_AUTO 100
#define PITCH_KD_V_AUTO 0.001
#define PITCH_INTEGRAL_V_AUTO 1500
#define PITCH_MAX_V_AUTO 30000
/* auto角度环 */
#define PITCH_KP_A_AUTO 0.25f
#define PITCH_KI_A_AUTO 0.0f
#define PITCH_KD_A_AUTO 0.001f
#define PITCH_INTEGRAL_A_AUTO 0.0f
#define PITCH_MAX_A_AUTO 20

#define CHASSIS_KP_V_FOLLOW 0.27f
#define CHASSIS_KI_V_FOLLOW 0
#define CHASSIS_KD_V_FOLLOW 0.001f
#define CHASSIS_INTEGRAL_V_FOLLOW 0
#define CHASSIS_MAX_V_FOLLOW 20

/******** DBUS底盘小陀螺相关设置 *******/
/** 旋转速度=截距+倍率*底盘功率限制/功率限制比例 **/
/* 旋转速度截距 */
#define ROTATE_INTERCEPT 4.5
/* 旋转速度倍率 */
#define ROTATE_MULTIPLIER 1.3
/* 功率限制比例 */
#define ROTATE_LIMIT_RATIO 80

/* 遥控器模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 0.5f
/* 云台yaw轴速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 0.5f

/* 鼠标键盘模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_PC_MOVE_RATIO_PIT 0.2f
/* 云台yaw轴速度 */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.3f

/* 归中必须在限定时间完成，否则等待重新关闭总开关。 */
#define GIMBAL_HOME_TIMEOUT_S 2.0f
#define GIMBAL_MAX_RATE_DEG_S 180.0f

#endif /* OMNI_JPL_GIMBAL_CONFIG_H */
