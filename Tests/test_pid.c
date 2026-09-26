#include "pid.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
static void near(float a,float b){assert(fabsf(a-b)<0.001f);}
int main(void){
    pid_obj_t p,q;
    pid_config_t c=INIT_PID_CONFIG(2,3,0,100,1000,PID_Trapezoid_Intergral|PID_Integral_Limit);
    pid_init(&p,&c);
    near(pid_calculate(&p,0,2,0.5f),7); near(p.Iout,3);
    near(pid_calculate(&p,0,4,0.5f),15.5f); near(p.Iout,7.5f);
    pid_clear(&p);near(p.Iout,0);near(p.Kp,2);
    c=(pid_config_t)INIT_PID_CONFIG(2,10,0,5,10,PID_Integral_Limit);
    pid_init(&p,&c);pid_init(&q,&c);
    near(pid_calculate(&p,0,100,0.01f),10);near(p.Iout,0);
    pid_calculate(&q,0,1,0.1f);near(q.Iout,1);near(p.Iout,0);
    for(int i=0;i<100;i++)pid_calculate(&q,0,1,0.1f);
    assert(q.Iout<=5);
    c=(pid_config_t)INIT_PID_CONFIG(0,0,2,10,10000,PID_Derivative_On_Measurement);
    pid_init(&p,&c);near(pid_calculate(&p,0,100,0.01f),0);
    near(pid_calculate(&p,0,200,0.01f),0);
    near(pid_calculate(&p,5,200,0.01f),-1000);
    c.Improve|=PID_DerivativeFilter;c.Derivative_LPF_RC=0.01f;
    pid_init(&p,&c);pid_calculate(&p,0,100,0.01f);
    near(pid_calculate(&p,5,100,0.01f),-500);
    c=(pid_config_t)INIT_PID_CONFIG(1,0,0,10,1000,PID_OutputFilter);
    c.Output_LPF_RC=0.09f;pid_init(&p,&c);
    near(pid_calculate(&p,0,100,0.01f),10);
    c=(pid_config_t)INIT_PID_CONFIG(0,1,0,100,1000,PID_ChangingIntegrationRate);
    c.CoefA=10;c.CoefB=5;pid_init(&p,&c);p.Iout=1;
    pid_calculate(&p,0,10,1);near(p.Iout,6);
    pid_calculate(&p,0,20,1);near(p.Iout,6);
    p.CoefA=0;pid_calculate(&p,0,20,1);near(p.Iout,6);
    c=(pid_config_t)INIT_PID_CONFIG(2,0,0,100,1000,PID_Proportional_On_Measurement);
    pid_init(&p,&c);near(pid_calculate(&p,0,10,0.01f),20);
    near(pid_calculate(&p,0,20,0.01f),20);
    near(pid_calculate(&p,2,20,0.01f),16);
    c=(pid_config_t)INIT_PID_CONFIG(1,1,0,100,1000,PID_ErrorHandle);
    pid_init(&p,&c);
    for(int i=0;i<501;i++)pid_calculate(&p,0,100,0.002f);
    assert(p.ERRORHandler.error_type==PID_MOTOR_BLOCKED_ERROR);
    pid_clear(&p);assert(p.ERRORHandler.error_type==PID_ERROR_NONE);
    p.DeadBand=2;p.Iout=10;near(pid_calculate(&p,0,1,0.01f),0);near(p.Iout,0);
    near(pid_calculate(&p,0,100,0),0);assert(!p.initialized);
    near(pid_calculate(&p,NAN,100,0.01f),0);
    puts("PASS: restored PID options, independent instances, saturation, deadband, dt and reset");
}
