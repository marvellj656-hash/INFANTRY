#include "gimbal/gimbal_control.h"
#include "gimbal/gimbal_config.h"
#include "shoot/shoot_control.h"
#include "shoot/shoot_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void)
{
    /* 云台详细轨迹与旧源码逐点对比见test_rtt_control.c。 */
    assert(fabsf(gimbal_encoder_relative(2,8190,1)-4*360.0f/8192)<0.001f);
    ShootController s;ShootOutput so;shoot_control_reset(&s);
    ShootInput si={.valid=true};ShootCommand sc={.enabled=true,.friction=true,.fire=true,.mode=SHOOT_SINGLE};
    shoot_control_update(&s,&sc,&si,true,0.002f,&so);assert(!so.armed);
    sc.fire=false;shoot_control_update(&s,&sc,&si,true,0.002f,&so);
    si.rpm[0]=FRICTION_LEFT_DIRECTION*FRICTION_RPM;si.rpm[1]=FRICTION_RIGHT_DIRECTION*FRICTION_RPM;
    for(unsigned i=0;i<400;i++)shoot_control_update(&s,&sc,&si,true,0.002f,&so);
    assert(so.friction_ready);
    sc.fire=true;shoot_control_update(&s,&sc,&si,true,0.002f,&so);
    float single=s.target_angle;assert(single==TRIGGER_STEP_DEG);
    for(unsigned i=0;i<10;i++)shoot_control_update(&s,&sc,&si,true,0.002f,&so);
    assert(s.target_angle==single); /* Holding trigger does not repeat single shots. */
    si.trigger_angle_deg=single;sc.fire=false;shoot_control_update(&s,&sc,&si,true,0.002f,&so);
    sc.mode=SHOOT_BURST3;shoot_control_update(&s,&sc,&si,true,0.002f,&so);
    sc.fire=true;shoot_control_update(&s,&sc,&si,true,0.002f,&so);
    assert(s.target_angle==single+3*TRIGGER_STEP_DEG);
    sc.mode=SHOOT_CONTINUOUS;
    shoot_control_update(&s,&sc,&si,true,0.002f,&so);
    assert(so.target_rpm[2]==TRIGGER_CONTINUOUS_RPM&&so.raw[2]<=10000);
    si.trigger_current_raw=9600;
    for(unsigned i=0;i<32;i++)shoot_control_update(&s,&sc,&si,true,0.002f,&so);
    assert(so.reversing);
    for(unsigned i=0;i<80;i++)shoot_control_update(&s,&sc,&si,true,0.002f,&so);
    assert(so.jam_latched&&so.raw[2]==0);
    shoot_control_update(&s,&sc,&si,false,0.002f,&so);
    assert(!so.armed&&so.raw[0]==0&&so.raw[1]==0&&so.raw[2]==0);
    sc.fire=true;shoot_control_update(&s,&sc,&si,true,0.002f,&so);assert(!so.armed);
    puts("PASS: gimbal homing/timeout/limits/modes, friction readiness, single/burst/continuous, jam, rearm");
}
