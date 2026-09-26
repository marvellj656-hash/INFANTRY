#include "gimbal/gimbal_control.h"
#include "shoot/shoot_control.h"
#include "shoot/shoot_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void)
{
    GimbalController g;GimbalOutput go;
    gimbal_control_reset(&g);
    GimbalCommand gc={.enabled=true,.neutral=true};
    GimbalInput gi={.valid=true};
    assert(fabsf(gimbal_encoder_relative(2,8190,1)-4*360.0f/8192)<0.001f);
    for(unsigned i=0;i<60;i++)gimbal_control_update(&g,&gc,&gi,true,0.002f,&go);
    assert(go.centered&&go.mode==GIMBAL_GYRO);
    gc.yaw_rate_deg_s=100;
    gimbal_control_update(&g,&gc,&gi,true,0.002f,&go);
    assert(go.target_deg[0]>0&&go.raw[0]<0);
    gimbal_control_update(&g,&gc,&gi,false,0.002f,&go);
    assert(go.raw[0]==0&&go.raw[1]==0&&!go.centered);
    gc.neutral=false;gimbal_control_update(&g,&gc,&gi,true,0.002f,&go);assert(go.mode==GIMBAL_RELAX);
    gc.neutral=true;gi.relative_deg[0]=30;
    for(unsigned i=0;i<1100;i++)gimbal_control_update(&g,&gc,&gi,true,0.002f,&go);
    assert(go.mode==GIMBAL_FAULT&&go.raw[0]==0);
    gc.enabled=false;gimbal_control_update(&g,&gc,&gi,true,0.002f,&go);
    assert(!g.fault_latched);
    gi.relative_deg[0]=0;gc.enabled=true;gc.yaw_rate_deg_s=0;
    for(unsigned i=0;i<60;i++)gimbal_control_update(&g,&gc,&gi,true,0.002f,&go);
    gc.auto_aim=true;gc.auto_pitch_deg=100;
    gimbal_control_update(&g,&gc,&gi,true,0.002f,&go);
    assert(go.mode==GIMBAL_AUTO&&go.target_deg[1]<=30);
    gi.relative_deg[1]=31;
    gimbal_control_update(&g,&gc,&gi,true,0.002f,&go);assert(go.raw[1]<=0);
    gi.angle_deg[0]=NAN;
    gimbal_control_update(&g,&gc,&gi,true,0.002f,&go);assert(go.raw[0]==0);
    ShootController s;ShootOutput so;shoot_control_reset(&s);
    ShootInput si={.valid=true};ShootCommand sc={.enabled=true,.friction=true,.fire=true,.mode=SHOOT_SINGLE};
    shoot_control_update(&s,&sc,&si,true,0.002f,&so);assert(!so.armed);
    sc.fire=false;shoot_control_update(&s,&sc,&si,true,0.002f,&so);
    si.rpm[0]=FRICTION_RPM;si.rpm[1]=-FRICTION_RPM;
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
