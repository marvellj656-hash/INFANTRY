#include "shoot/shoot_control.h"
#include "shoot/shoot_config.h"
#include "chassis_follow.h"
#include "chassis/chassis_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void zero(const ShootOutput *out)
{
    for(unsigned i=0;i<3;i++) assert(out->raw[i]==0);
}
static void run(ShootController *s,ShootCommand *cmd,ShootInput *in,ShootOutput *out)
{
    *cmd=(ShootCommand){.enabled=true,.friction=true,.mode=SHOOT_SINGLE};
    *in=(ShootInput){.valid=true};
    shoot_control_reset(s);
    for(unsigned i=0;i<4;i++) shoot_control_update(s,cmd,in,true,0.002f,out);
    assert(out->armed);
    in->rpm[0]=FRICTION_LEFT_DIRECTION*FRICTION_RPM;
    in->rpm[1]=FRICTION_RIGHT_DIRECTION*FRICTION_RPM;
    cmd->friction=false;
}
static void test_brake(void)
{
    ShootController s;ShootCommand c;ShootInput in;ShootOutput out;
    run(&s,&c,&in,&out);
    c.fire=true; /* stop overrides held trigger */
    shoot_control_update(&s,&c,&in,true,0.002f,&out);
    assert(out.braking && !out.friction_ready && out.raw[2]==0);
    for(unsigned i=0;i<2;i++) {
        assert(out.raw[i]*in.rpm[i]<0);
        assert(fabsf(out.raw[i])<=FRICTION_BRAKE_LIMIT_RAW);
    }
    /* A bounded toy plant only checks feedback direction, not real braking time. */
    for(unsigned n=0;n<400 && out.braking;n++) {
        for(unsigned i=0;i<2;i++) {
            in.rpm[i] *= 0.96f;
            if(n==150) in.rpm[i]=-in.rpm[i]; /* sign reversal near rest must not propel */
        }
        shoot_control_update(&s,&c,&in,true,0.002f,&out);
        for(unsigned i=0;i<2;i++) assert(out.raw[i]*in.rpm[i]<=0);
        assert(out.raw[2]==0);
    }
    assert(!out.braking && !out.brake_timed_out);zero(&out);
    in.rpm[0]=2000; /* already stopped: external motion doesn't restart brake */
    shoot_control_update(&s,&c,&in,true,0.002f,&out);zero(&out);

    run(&s,&c,&in,&out);
    bool timed_out=false;
    for(unsigned n=0;n<800;n++) {
        shoot_control_update(&s,&c,&in,true,0.002f,&out);
        timed_out |= out.brake_timed_out;
    }
    assert(timed_out && !out.braking);zero(&out);

    run(&s,&c,&in,&out);
    shoot_control_update(&s,&c,&in,true,0.002f,&out);
    c.enabled=false;
    shoot_control_update(&s,&c,&in,true,0.002f,&out);zero(&out);assert(!out.braking);
    c.enabled=true;shoot_control_update(&s,&c,&in,true,0.002f,&out);zero(&out);
    run(&s,&c,&in,&out);
    shoot_control_update(&s,&c,&in,true,0.002f,&out);
    shoot_control_update(&s,&c,&in,false,0.002f,&out);zero(&out);
    shoot_control_update(&s,&c,&in,true,0.002f,&out);zero(&out);
    run(&s,&c,&in,&out);
    shoot_control_update(&s,&c,&in,true,0.002f,&out);
    c.friction=true;c.fire=true;
    shoot_control_update(&s,&c,&in,true,0.002f,&out);zero(&out);assert(!out.armed);
    c.fire=false;
    shoot_control_update(&s,&c,&in,true,0.002f,&out);
    shoot_control_update(&s,&c,&in,true,0.002f,&out);
    assert(out.armed && !out.friction_ready && out.raw[2]==0);
    c.friction=false;in.rpm[1]=NAN;
    shoot_control_update(&s,&c,&in,true,0.002f,&out);zero(&out);
    run(&s,&c,&in,&out);
    shoot_control_update(&s,&c,&in,true,0.03f,&out);zero(&out);
}
static void test_follow(void)
{
    ChassisFollow s;ChassisFollowOutput out;
    const pid_config_t cfg=CHASSIS_FOLLOW_PID_CONFIG;
    assert(cfg.Kp==CHASSIS_FOLLOW_KP && cfg.Ki==CHASSIS_FOLLOW_KI);
    assert(cfg.Kd==CHASSIS_FOLLOW_KD);
    assert(cfg.IntegralLimit==CHASSIS_FOLLOW_I_LIMIT_RAD_S);
    assert(cfg.MaxOut==CHASSIS_FOLLOW_MAX_RAD_S);
    /* 算法算例使用固定题目，不把实车调参当成回归失败。
     * 上面检查项目配置；下面独立验证已知P增益、死区和限幅的计算。 */
    const pid_config_t scenario=INIT_PID_CONFIG(0.04f,0,0,0,1.5f,
        PID_Derivative_On_Measurement | PID_Integral_Limit);
    chassis_follow_init(&s,&scenario);
    chassis_follow_update(&s,20,2,true,0.002f,&out);
    assert(out.active && fabsf(out.wz_rad_s-0.72f)<0.0001f);
    chassis_follow_update(&s,-20,2,true,0.002f,&out);assert(out.wz_rad_s<0);
    chassis_follow_update(&s,359,2,true,0.002f,&out);
    assert(out.active && out.wz_rad_s==0 && out.error_deg==-1);
    chassis_follow_update(&s,181,2,true,0.002f,&out);
    assert(out.error_deg==-179 && out.wz_rad_s==-scenario.MaxOut);
    chassis_follow_update(&s,90,2,false,0.002f,&out);assert(!out.active && out.wz_rad_s==0);
    chassis_follow_update(&s,NAN,2,true,0.002f,&out);assert(!out.active);
    chassis_follow_update(&s,20,2,true,0.021f,&out);assert(!out.active);
    chassis_follow_update(&s,20,-1,true,0.002f,&out);assert(!out.active);
    float angle=30;
    for(unsigned i=0;i<1000;i++) {
        chassis_follow_update(&s,angle,2,true,0.002f,&out);
        float next=angle-out.wz_rad_s*0.002f*57.2957795f;
        assert(next<=angle && next>=0);angle=next;
    }
    assert(angle<2.5f);
}
/* 固定误差只测试积分能否积累、限幅和清零，不模拟整车响应。 */
static void test_follow_integral(void)
{
    ChassisFollow s; ChassisFollowOutput out;
    pid_config_t cfg=CHASSIS_FOLLOW_PID_CONFIG;
    cfg.Kp=0.04f; cfg.Ki=1.0f; cfg.Kd=0;
    cfg.IntegralLimit=0; cfg.MaxOut=1.5f;
    chassis_follow_init(&s,&cfg);
    for(unsigned n=0;n<100;n++)
        chassis_follow_update(&s,12,2,true,0.002f,&out);
    assert(s.pid.Iout==0);
    assert(fabsf(out.wz_rad_s-0.4f)<0.0001f);
    cfg.IntegralLimit=0.15f;
    chassis_follow_init(&s,&cfg);
    chassis_follow_update(&s,12,2,true,0.002f,&out);
    assert(s.pid.Iout>0 && s.pid.Iout<cfg.IntegralLimit);
    for(unsigned n=0;n<100;n++) {
        chassis_follow_update(&s,12,2,true,0.002f,&out);
        assert(s.pid.Iout<=cfg.IntegralLimit);
    }
    assert(fabsf(s.pid.Iout-0.15f)<0.0001f);
    assert(fabsf(out.wz_rad_s-0.55f)<0.0001f);
    chassis_follow_update(&s,-12,2,true,0.002f,&out);
    assert(s.pid.Iout<0.15f);
    chassis_follow_update(&s,12,2,false,0.002f,&out);
    assert(!out.active && s.pid.Iout==0);
    chassis_follow_update(&s,12,2,true,0.002f,&out);
    assert(s.pid.Iout>0);
    chassis_follow_update(&s,1,2,true,0.002f,&out);
    assert(out.active && out.wz_rad_s==0 && s.pid.Iout==0);
    cfg.MaxOut=0.2f;
    chassis_follow_init(&s,&cfg);
    for(unsigned n=0;n<100;n++)
        chassis_follow_update(&s,12,2,true,0.002f,&out);
    assert(s.pid.Iout==0 && out.wz_rad_s==cfg.MaxOut);
}
int main(void)
{
    test_brake();test_follow();test_follow_integral();
    puts("PASS: brake direction/limit/settling/timeout/disable/loss/rearm; follow sign/wrap/deadband/bounds/invalid reference/configured integral limit");
}
