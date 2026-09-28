#include "gimbal/gimbal_control.h"
#include "gimbal/gimbal_config.h"
#include "chassis/chassis_config.h"
#include "chassis_follow.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
float rtt_reference_pid(pid_obj_t *,float,float,float);
static void near(float a,float b) { assert(fabsf(a-b)<0.01f); }
int main(void) {
 GimbalController g;gimbal_control_reset(&g);
 pid_obj_t configs[9]={g.angle[0],g.angle[1],g.speed[0],g.speed[1]};
 GimbalCommand ac={.enabled=true,.auto_aim=true};GimbalInput ai={.valid=true};GimbalOutput ao;
 g.mode=GIMBAL_GYRO;gimbal_control_update(&g,&ac,&ai,true,0.001f,&ao);
 configs[5]=g.angle[0];configs[6]=g.angle[1];configs[7]=g.speed[0];configs[8]=g.speed[1];
 for(unsigned i=5;i<9;i++)pid_clear(&configs[i]);
 pid_config_t fc=CHASSIS_FOLLOW_PID_CONFIG;pid_init(&configs[4],&fc);
 for(unsigned axis=0;axis<9;axis++) {
   pid_obj_t a=configs[axis],b=a;
   for(unsigned n=0;n<2000;n++) {
     float x=sinf(n*0.03f)*15, y=cosf(n*0.04f)*20, dt=(n%3+1)*0.001f;
     if(n%9==0)y=x; /* 零误差不清积分，也不输出积分。 */
     if(n==1000){pid_clear(&a);pid_clear(&b);}
     near(pid_calculate(&a,x,y,dt),rtt_reference_pid(&b,x,y,dt));
     near(a.Iout,b.Iout);near(a.Dout,b.Dout);
   }
 }
 GimbalCommand c={.enabled=true,.neutral=false};
 GimbalInput in={.valid=true,.relative_deg={30,10},.angle_deg={100,8}};GimbalOutput o;
 gimbal_control_reset(&g);
 gimbal_control_update(&g,&c,&in,true,0.001f,&o);
 assert(o.mode==GIMBAL_INIT);near(o.target_deg[1],9.9f);
 in.relative_deg[1]=5;
 gimbal_control_update(&g,&c,&in,true,0.001f,&o);near(o.target_deg[1],4.9f);
 in.relative_deg[0]=in.relative_deg[1]=0;
 gimbal_control_update(&g,&c,&in,true,0.001f,&o);
 assert(o.centered);near(o.target_deg[0],100);near(o.target_deg[1],0);
 gimbal_control_update(&g,&c,&in,true,0.001f,&o);
 assert(o.raw[1]!=0); /* INS仍偏8度时不能因为编码器归中而失去控制。 */
 c.auto_aim=true;c.auto_yaw_deg=110;c.auto_pitch_deg=4;
 gimbal_control_update(&g,&c,&in,true,0.001f,&o);
 assert(o.mode==GIMBAL_AUTO);near(o.target_deg[0],110);
 c.auto_aim=false;
 gimbal_control_update(&g,&c,&in,true,0.001f,&o);near(o.target_deg[0],110);
 gimbal_control_update(&g,&c,&in,false,0.001f,&o);assert(o.raw[0]==0&&o.raw[1]==0);
 gimbal_control_update(&g,&c,&in,true,0.001f,&o);assert(o.mode==GIMBAL_GYRO);near(o.target_deg[0],110);
 c.auto_aim=true;c.auto_pitch_deg=100;
 in.relative_deg[1]=31;in.angle_deg[1]=0;
 gimbal_control_update(&g,&c,&in,true,0.001f,&o);near(o.target_deg[1],30);assert(o.raw[1]>0);
 c.enabled=false;gimbal_control_update(&g,&c,&in,true,0.001f,&o);assert(o.raw[1]==0&&o.mode==GIMBAL_RELAX);
 c.enabled=true;c.auto_aim=false;in.relative_deg[0]=30;in.relative_deg[1]=10;
 for(unsigned n=0;n<2100;n++)gimbal_control_update(&g,&c,&in,true,0.001f,&o);
 assert(o.mode==GIMBAL_INIT); /* RTT超时仍未到位则继续归中，不锁死。 */
 in.relative_deg[0]=in.relative_deg[1]=7;
 gimbal_control_update(&g,&c,&in,true,0.001f,&o);assert(o.centered);
 puts("PASS: RTT PID oracle 18000 samples; home ramp/tolerance, INS handoff, mode continuity, loss/recovery and target limits");
}
