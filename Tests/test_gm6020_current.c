#include "motor/dji_motor_service.h"
#include "gimbal/gimbal_control.h"
#include "gimbal/gimbal_config.h"
#include "bsp_can.h"
#include <assert.h>
#include <stdio.h>
static uint32_t now=1;
static BspCanFrame frames[2][5];
static unsigned count[2];
uint32_t bsp_time_ms(void) { return now; }
bool bsp_can_is_ready(uint8_t bus) { return bus==1 || bus==2; }
bool bsp_can_send_latest(BspCanTx *tx,const BspCanFrame *f,volatile CanTxDebug *d) {
    (void)tx; (void)d; assert(count[f->bus-1]<5);
    frames[f->bus-1][count[f->bus-1]++]=*f; return true;
}
static const BspCanFrame *frame(unsigned bus,unsigned id) {
    for(unsigned i=0;i<count[bus-1];i++) if(frames[bus-1][i].id==id) return &frames[bus-1][i];
    assert(0); return NULL;
}
static int slot(const BspCanFrame *f,unsigned i) {
    unsigned v=((unsigned)f->data[2*i]<<8)|f->data[2*i+1];
    return v>=32768 ? (int)v-65536 : (int)v;
}
int main(void) {
    /* 真电机层+真控制器，只有时钟/CAN换成记录器。 */
    DjiMotorConfig c={MOTOR_GIMBAL_YAW,DJI_GM6020_CURRENT,1,3,30000,100,20};
    assert(!dji_motor_register(&c)); /* 电流模式禁止电压量程。 */
    c.output_limit_raw=8192; DjiMotorHandle yaw=dji_motor_register(&c); assert(yaw);
    c.id=MOTOR_GIMBAL_PITCH;c.bus=2;c.esc_id=4;DjiMotorHandle pitch=dji_motor_register(&c);assert(pitch);
    c.id=MOTOR_TRIGGER;c.bus=1;c.esc_id=7;DjiMotorHandle high=dji_motor_register(&c);assert(high);
    c.id=MOTOR_CHASSIS_FL;c.type=DJI_M3508;c.esc_id=1;DjiMotorHandle wheel=dji_motor_register(&c);assert(wheel);
    c.id=MOTOR_CHASSIS_FR;c.type=DJI_GM6020;c.bus=2;c.esc_id=1;DjiMotorHandle voltage=dji_motor_register(&c);assert(voltage);
    uint8_t feedback[8]={0,0,0,0,0,0,30,0};
    assert(dji_motor_receive(1,0x207,feedback,8,now));
    assert(dji_motor_receive(2,0x208,feedback,8,now));
    assert(dji_motor_receive(1,0x20b,feedback,8,now));
    assert(dji_motor_receive(1,0x201,feedback,8,now));
    assert(dji_motor_receive(2,0x205,feedback,8,now));
    assert(dji_motor_set_output(yaw,30000));assert(dji_motor_set_output(pitch,-30000));
    assert(dji_motor_set_output(high,1234));assert(dji_motor_set_output(wheel,456));
    assert(dji_motor_set_output(voltage,789));dji_motor_service();
    assert(slot(frame(1,0x1fe),2)==8192 && slot(frame(2,0x1fe),3)==-8192);
    assert(slot(frame(1,0x2fe),2)==1234 && slot(frame(1,0x2fe),3)==0);
    assert(slot(frame(1,0x200),0)==456 && slot(frame(2,0x1ff),0)==789);
    for(unsigned i=0;i<4;i++) if(i!=2) assert(slot(frame(1,0x1fe),i)==0);
    assert(debug_motor[MOTOR_GIMBAL_YAW].tx_id==0x1fe);
    now+=20;count[0]=count[1]=0;dji_motor_service();
    for(unsigned b=0;b<2;b++) for(unsigned i=0;i<count[b];i++)
        for(unsigned j=0;j<8;j++) assert(frames[b][i].data[j]==0);
    /* 电流换算、正负方向、饱和和disable均走实际双环。 */
    GimbalController g;GimbalOutput out;GimbalInput in={.valid=true};
    GimbalCommand cmd={.enabled=true,.neutral=true};gimbal_control_reset(&g);
    for(unsigned i=0;i<110;i++) gimbal_control_update(&g,&cmd,&in,true,0.001f,&out);
    assert(out.mode==GIMBAL_GYRO);
    in.rate_rad_s[0]=1000;in.rate_rad_s[1]=1000;
    gimbal_control_update(&g,&cmd,&in,true,0.001f,&out);
    assert(out.raw[0]==GIMBAL_CURRENT_LIMIT_RAW && out.raw[1]==-GIMBAL_CURRENT_LIMIT_RAW);
    cmd.enabled=false;gimbal_control_update(&g,&cmd,&in,true,0.001f,&out);
    assert(out.raw[0]==0 && out.raw[1]==0);
    puts("PASS: GM6020 current low/high groups, both buses, mixed voltage/current, clamp, timeout, cascade output");
}
