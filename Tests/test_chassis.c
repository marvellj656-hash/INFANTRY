#include "chassis_control.h"
#include "chassis_config.h"
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
    float feedback[4] = {0}; ChassisOutput o;
    chassis_step(s, (ChassisVelocity){0}, feedback, true, true, 0.002f, 30, &o);
    assert(s->armed);
}
int main(void)
{
    ChassisVelocity target = chassis_target(660, 0, 660);
    near(target.vx, 300, 0.001f); near(target.wz, 0.5f, 0.001f);
    target = chassis_target(330, 0, 0);
    assert(target.vx > 140 && target.vx < 151); /* Full stick range retained. */
    target = chassis_target(660, 660, 0);
    near(hypotf(target.vx, target.vy), 300, 0.001f);
    target = chassis_target(-5, 5, 0);
    assert(target.vx == 0 && target.vy == 0 && target.wz == 0);
    float rpm[4];
    chassis_inverse((ChassisVelocity){100,0,0}, rpm);
    assert(rpm[0]>0 && rpm[1]>0 && rpm[2]<0 && rpm[3]<0);
    chassis_inverse((ChassisVelocity){0,100,0}, rpm);
    assert(rpm[0]>0 && rpm[1]<0 && rpm[2]<0 && rpm[3]>0);
    chassis_inverse((ChassisVelocity){0,0,0.5f}, rpm);
    near(rpm[0], rpm[3], 0.001f); assert(rpm[0] > 0);
    chassis_inverse((ChassisVelocity){3000,3000,5}, rpm);
    for (unsigned i=0; i<4; ++i) assert(fabsf(rpm[i]) <= 1000.001f);

    ChassisController fast = {0}, slow = {0}; ChassisOutput a,b;
    float stopped[4] = {0}; target = (ChassisVelocity){300,0,0.5f};
    chassis_step(&fast, target, stopped, true, false, 0.002f, 30, &a);
    all_zero(&a); /* Cannot arm with stick displaced. */
    arm(&fast); arm(&slow);
    for (int i=0; i<300; ++i) {
        chassis_step(&fast, target, stopped, true, false, 0.002f, 30, &a);
        for (unsigned m=0; m<4; ++m) assert(abs(a.current_raw[m]) <= 2000);
    }
    for (int i=0; i<60; ++i)
        chassis_step(&slow, target, stopped, true, false, 0.010f, 30, &b);
    near(a.velocity.vx,300,0.02f); near(a.velocity.vx,b.velocity.vx,0.02f);
    near(a.velocity.wz,0.5f,0.002f);
    for (int i=0; i<600; ++i)
        chassis_step(&fast, (ChassisVelocity){-300,0,0}, stopped, true, false, 0.002f,30,&a);
    near(a.velocity.vx,-300,0.02f);
    chassis_step(&fast,target,stopped,false,false,0.002f,30,&a);
    all_zero(&a); assert(fast.pid[0].integral == 0 && fast.velocity.vx == 0);
    chassis_step(&fast,target,stopped,true,false,0.002f,30,&a);
    all_zero(&a); /* Restore does not immediately resume motion. */
    arm(&fast);
    chassis_step(&fast,target,stopped,true,false,0.03f,30,&a);
    all_zero(&a); /* Delayed control cycle cannot produce a large step. */
    arm(&fast);
    chassis_step(&fast,(ChassisVelocity){NAN,0,0},stopped,true,false,0.002f,30,&a);
    all_zero(&a);

    float raw[4] = {2000,-2000,1000,-1000};
    float speed[4] = {2000,-2000,2000,-2000}; float estimate;
    float scale = chassis_power_limit(raw,speed,8,&estimate);
    assert(scale > 0 && scale < 1 && estimate <= 8.001f);
    near(raw[0],-raw[1],0.001f); near(raw[0],raw[2]*2,0.001f);
    raw[0] = NAN;
    assert(chassis_power_limit(raw,speed,8,&estimate) == 0);
    for (int i=0;i<4;++i) assert(raw[i] == 0);
    raw[0] = 1000;
    assert(chassis_power_limit(raw,speed,0,&estimate) == 0 && raw[0] == 0);

    int16_t currents[4] = {2000,-2000,32767,-32768}; uint8_t wire[8];
    chassis_pack_current(currents,wire);
    const uint8_t expected[8] = {0x07,0xd0,0xf8,0x30,0x07,0xd0,0xf8,0x30};
    assert(memcmp(wire,expected,8) == 0);
    puts("PASS: omni wheel mapping, RC range, time-based ramps, PID limits, stop/rearm, power scaling, CAN packing");
    return 0;
}
