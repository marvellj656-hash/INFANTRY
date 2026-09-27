#ifndef SHOOT_CONTROL_H
#define SHOOT_CONTROL_H
#include "pid.h"
#include <stdint.h>
#include <stdbool.h>
typedef enum
{
    SHOOT_STOP,
    SHOOT_SINGLE,
    SHOOT_BURST3,
    SHOOT_CONTINUOUS,
    SHOOT_REVERSE
} ShootMode;
typedef struct
{
    bool enabled, friction, fire;
    ShootMode mode;
    uint32_t received_at_ms;
} ShootCommand;
typedef struct
{
    float rpm[3], trigger_angle_deg; /* [左摩擦轮、右摩擦轮、拨弹]；角度是转子多圈度数。 */
    int16_t trigger_current_raw;
    bool valid;
} ShootInput;
typedef struct
{
    pid_obj_t speed[3], angle;
    pid_obj_t brake[2]; /* 独立的停车PID，不能沿用运行时的积分和微分历史。 */
    float friction_rpm, ready_s, target_angle, jam_s, reverse_s;
    ShootMode last_mode;
    bool armed, last_fire, jam_latched;
    bool friction_running, braking;
    float brake_s, stopped_s;
} ShootController;
typedef struct
{
    int16_t raw[3];
    float target_rpm[3];
    bool armed, friction_ready, reversing, jam_latched;
    bool braking, brake_timed_out;
} ShootOutput;
void shoot_control_reset(ShootController *s);
void shoot_control_update(ShootController *s, const ShootCommand *cmd, const ShootInput *in,
                          bool permit, float dt, ShootOutput *out);
#endif
