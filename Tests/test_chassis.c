#include "chassis_task.h"
#include "chassis_config.h"
#include "cmd/cmd_task.h"
#include "power_model.h"
#include "motor/dji_protocol.h"
static const OmniGeometry geometry = {471,19,230,1000};
static const PowerModel model = {POWER_RAW_TO_A,POWER_ROTOR_KT,POWER_EQUIVALENT_R,POWER_IDLE_W};
static const int16_t limits[4] = {2000,2000,2000,2000};
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static void near(float a, float b, float tolerance) { assert(fabsf(a-b) <= tolerance); }
static void all_zero(const ChassisOutput *o) {
    assert(!o->armed);
    for (unsigned i=0; i<4; ++i) assert(o->current_raw[i] == 0);
}
static void arm(ChassisController *s) {
    chassis_control_reset(s);
    float feedback[4] = {0}; ChassisOutput o;
    chassis_control_update(s, (ChassisVelocity){0}, feedback, true, true, 0.002f, 30, &o);
    assert(s->armed);
}
int main(void)
{
    ChassisVelocity target = cmd_build_chassis_command(660, 0, 660);
    near(target.vx, CHASSIS_MAX_SPEED_MM_S, 0.001f); near(target.wz, CHASSIS_MAX_YAW_RAD_S, 0.001f);
    target = cmd_build_chassis_command(330, 0, 0);
    near(target.vx, CHASSIS_MAX_SPEED_MM_S*325.0f/655.0f,0.001f); /* Full stick range retained. */
    target = cmd_build_chassis_command(660, 660, 0);
    near(hypotf(target.vx, target.vy), CHASSIS_MAX_SPEED_MM_S, 0.001f);
    target = cmd_build_chassis_command(-5, 5, 0);
    assert(target.vx == 0 && target.vy == 0 && target.wz == 0);
    float rpm[4];
    omni_inverse_kinematics((ChassisVelocity){100,0,0}, &geometry, rpm);
    assert(rpm[0]>0 && rpm[1]>0 && rpm[2]<0 && rpm[3]<0);
    omni_inverse_kinematics((ChassisVelocity){0,100,0}, &geometry, rpm);
    assert(rpm[0]>0 && rpm[1]<0 && rpm[2]<0 && rpm[3]>0);
    omni_inverse_kinematics((ChassisVelocity){0,0,0.5f}, &geometry, rpm);
    near(rpm[0], rpm[3], 0.001f); assert(rpm[0] > 0);
    omni_inverse_kinematics((ChassisVelocity){3000,3000,5}, &geometry, rpm);
    for (unsigned i=0; i<4; ++i) assert(fabsf(rpm[i]) <= 1000.001f);


    /* 实物量纲检查：100转/分钟的转子并不等于100rad/s，也不等于轮轴RPM。 */
    float known_rpm[4] = {100,100,-100,-100};
    ChassisVelocity measured = omni_forward_kinematics(known_rpm,&geometry);
    near(measured.vx, 100.0f*471.0f/(60.0f*19.0f*0.70710678f),0.001f);
    near(measured.vy,0,0.001f); near(measured.wz,0,0.001f);
    for (int n=0;n<4;++n) known_rpm[n]=100;
    measured=omni_forward_kinematics(known_rpm,&geometry);
    near(measured.vx,0,0.001f); near(measured.vy,0,0.001f);
    near(measured.wz,100.0f*471.0f/(60.0f*19.0f*230.0f),0.001f);
    for(int x=-100;x<=100;x+=50) for(int y=-100;y<=100;y+=50) {
        ChassisVelocity input={(float)x,(float)y,0.2f};
        omni_inverse_kinematics(input,&geometry,rpm);
        measured=omni_forward_kinematics(rpm,&geometry);
        near(measured.vx,input.vx,0.001f);near(measured.vy,input.vy,0.001f);near(measured.wz,input.wz,0.001f);
    }
    /* 遥控停机不应抹掉真实轮速；非有限反馈则不能发布有效车速。 */
    ChassisController parked={0};ChassisOutput parked_out;
    chassis_control_update(&parked,(ChassisVelocity){0},known_rpm,false,true,0.002f,30,&parked_out);
    assert(!parked_out.armed && parked_out.measured_velocity_valid);
    assert(parked_out.measured_velocity.wz>0);
    known_rpm[0]=NAN;
    chassis_control_update(&parked,(ChassisVelocity){0},known_rpm,false,true,0.002f,30,&parked_out);
    assert(!parked_out.measured_velocity_valid);

    ChassisController fast = {0}, slow = {0}; ChassisOutput a,b;
    float stopped[4] = {0}; target = (ChassisVelocity){300,0,0.5f};
    chassis_control_update(&fast, target, stopped, true, false, 0.002f, 30, &a);
    all_zero(&a); /* Cannot arm with stick displaced. */
    arm(&fast); arm(&slow);
    for (int i=0; i<300; ++i) {
        chassis_control_update(&fast, target, stopped, true, false, 0.002f, 30, &a);
        for (unsigned m=0; m<4; ++m) assert(abs(a.current_raw[m]) <= CHASSIS_CURRENT_LIMIT_RAW);
    }
    for (int i=0; i<60; ++i)
        chassis_control_update(&slow, target, stopped, true, false, 0.010f, 30, &b);
    assert(a.current_raw[0] > 0); /* A passing trajectory must actually drive the wheel PID. */
    near(a.velocity.vx,300,0.02f); near(a.velocity.vx,b.velocity.vx,0.02f);
    near(a.velocity.wz,0.5f,0.002f);
    for (int i=0; i<600; ++i)
        chassis_control_update(&fast, (ChassisVelocity){-300,0,0}, stopped, true, false, 0.002f,30,&a);
    near(a.velocity.vx,-300,0.02f);
    chassis_control_update(&fast,target,stopped,false,false,0.002f,30,&a);
    all_zero(&a); assert(fast.pid[0].Iout == 0 && fast.velocity.vx == 0);
    chassis_control_update(&fast,target,stopped,true,false,0.002f,30,&a);
    all_zero(&a); /* Restore does not immediately resume motion. */
    arm(&fast);
    chassis_control_update(&fast,target,stopped,true,false,0.03f,30,&a);
    all_zero(&a); /* Delayed control cycle cannot produce a large step. */
    arm(&fast);
    chassis_control_update(&fast,(ChassisVelocity){NAN,0,0},stopped,true,false,0.002f,30,&a);
    all_zero(&a);

    float raw[4] = {2000,-2000,1000,-1000};
    float speed[4] = {2000,-2000,2000,-2000}; float estimate;
    float scale = power_model_limit(&model,raw,speed,8,&estimate);
    assert(scale > 0 && scale < 1 && estimate <= 8.001f);
    near(raw[0],-raw[1],0.001f); near(raw[0],raw[2]*2,0.001f);
    raw[0] = NAN;
    assert(power_model_limit(&model,raw,speed,8,&estimate) == 0);
    for (int i=0;i<4;++i) assert(raw[i] == 0);
    raw[0] = 1000;
    assert(power_model_limit(&model,raw,speed,0,&estimate) == 0 && raw[0] == 0);

    int16_t currents[4] = {2000,-2000,32767,-32768}; uint8_t wire[8];
    dji_motor_encode_currents(currents,limits,wire);
    const uint8_t expected[8] = {0x07,0xd0,0xf8,0x30,0x07,0xd0,0xf8,0x30};
    assert(memcmp(wire,expected,8) == 0);
    puts("PASS: omni wheel mapping, RC range, time-based ramps, PID limits, stop/rearm, power scaling, CAN packing");
    return 0;
}
