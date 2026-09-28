#ifndef CMD_CONFIG_H
#define CMD_CONFIG_H
/* 摇杆回中附近忽略小抖动；660来自DBUS的正常满行程。 */
#define CMD_STICK_MAX 660.0f
#define CMD_STICK_DEADBAND 5.0f
/* 满杆产生的目标角速度，单位度/秒；RTT每1ms增量换算：660 * 0.0009 * 0.5 / 0.001 = 297。 */
#define CMD_YAW_STICK_RATE_DEG_S 297.0f
#define CMD_PITCH_STICK_RATE_DEG_S 297.0f
/* 每一单位DBUS鼠标增量产生的角速度要求，单位(度/秒)/计数。 */
#define CMD_YAW_MOUSE_RATE_PER_COUNT 3.0f
#define CMD_PITCH_MOUSE_RATE_PER_COUNT 2.0f
/* 背面拨轮超过正/负门槛才触发发射/退弹，单位是通道原始值。 */
#define CMD_FIRE_WHEEL_THRESHOLD 200
#endif
