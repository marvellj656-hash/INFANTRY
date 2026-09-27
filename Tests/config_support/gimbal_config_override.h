#ifndef TEST_GIMBAL_CONFIG_OVERRIDE_H
#define TEST_GIMBAL_CONFIG_OVERRIDE_H
#include "gimbal/gimbal_config.h"
/* 仅主机测试强制包含；固件不包含本文件。 */
#undef YAW_KP_A_IMU
#define YAW_KP_A_IMU 1.0f
#undef YAW_KI_A_IMU
#define YAW_KI_A_IMU 2.0f
#undef YAW_KD_A_IMU
#define YAW_KD_A_IMU 3.0f
#undef YAW_INTEGRAL_A_IMU
#define YAW_INTEGRAL_A_IMU 4.0f
#undef YAW_MAX_A_IMU
#define YAW_MAX_A_IMU 5.0f
#undef YAW_KP_V_IMU
#define YAW_KP_V_IMU 6.0f
#undef YAW_KI_V_IMU
#define YAW_KI_V_IMU 7.0f
#undef YAW_KD_V_IMU
#define YAW_KD_V_IMU 8.0f
#undef YAW_INTEGRAL_V_IMU
#define YAW_INTEGRAL_V_IMU 9.0f
#undef YAW_MAX_V_IMU
#define YAW_MAX_V_IMU 10.0f
#undef PITCH_KP_A_IMU
#define PITCH_KP_A_IMU 11.0f
#undef PITCH_KI_A_IMU
#define PITCH_KI_A_IMU 12.0f
#undef PITCH_KD_A_IMU
#define PITCH_KD_A_IMU 13.0f
#undef PITCH_INTEGRAL_A_IMU
#define PITCH_INTEGRAL_A_IMU 14.0f
#undef PITCH_MAX_A_IMU
#define PITCH_MAX_A_IMU 15.0f
#undef PITCH_KP_V_IMU
#define PITCH_KP_V_IMU 16.0f
#undef PITCH_KI_V_IMU
#define PITCH_KI_V_IMU 17.0f
#undef PITCH_KD_V_IMU
#define PITCH_KD_V_IMU 18.0f
#undef PITCH_INTEGRAL_V_IMU
#define PITCH_INTEGRAL_V_IMU 19.0f
#undef PITCH_MAX_V_IMU
#define PITCH_MAX_V_IMU 20.0f
#undef YAW_KP_A_AUTO
#define YAW_KP_A_AUTO 21.0f
#undef YAW_KI_A_AUTO
#define YAW_KI_A_AUTO 22.0f
#undef YAW_KD_A_AUTO
#define YAW_KD_A_AUTO 23.0f
#undef YAW_INTEGRAL_A_AUTO
#define YAW_INTEGRAL_A_AUTO 24.0f
#undef YAW_MAX_A_AUTO
#define YAW_MAX_A_AUTO 25.0f
#undef YAW_KP_V_AUTO
#define YAW_KP_V_AUTO 26.0f
#undef YAW_KI_V_AUTO
#define YAW_KI_V_AUTO 27.0f
#undef YAW_KD_V_AUTO
#define YAW_KD_V_AUTO 28.0f
#undef YAW_INTEGRAL_V_AUTO
#define YAW_INTEGRAL_V_AUTO 29.0f
#undef YAW_MAX_V_AUTO
#define YAW_MAX_V_AUTO 30.0f
#undef PITCH_KP_A_AUTO
#define PITCH_KP_A_AUTO 31.0f
#undef PITCH_KI_A_AUTO
#define PITCH_KI_A_AUTO 32.0f
#undef PITCH_KD_A_AUTO
#define PITCH_KD_A_AUTO 33.0f
#undef PITCH_INTEGRAL_A_AUTO
#define PITCH_INTEGRAL_A_AUTO 34.0f
#undef PITCH_MAX_A_AUTO
#define PITCH_MAX_A_AUTO 35.0f
#undef PITCH_KP_V_AUTO
#define PITCH_KP_V_AUTO 36.0f
#undef PITCH_KI_V_AUTO
#define PITCH_KI_V_AUTO 37.0f
#undef PITCH_KD_V_AUTO
#define PITCH_KD_V_AUTO 38.0f
#undef PITCH_INTEGRAL_V_AUTO
#define PITCH_INTEGRAL_V_AUTO 39.0f
#undef PITCH_MAX_V_AUTO
#define PITCH_MAX_V_AUTO 40.0f
#endif
