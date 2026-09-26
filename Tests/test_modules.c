
#include "remote/remote.h"
#include "motor/dji_motor_service.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t now, generation;
static bool rx_ok=true, packet_ready, restart;
static BspUartPacket packet;
static BspCanFrame sent;
static unsigned sends;
static bool tx_ok=true, bus_ready=true;
bool bsp_can_is_ready(uint8_t bus) { (void)bus; return bus_ready; }
uint32_t bsp_time_ms(void) { return now; }
bool bsp_uart_init(void) { return true; }
bool bsp_uart_recover_rx(void) { if(restart){generation++;restart=false;packet_ready=false;} return rx_ok; }
uint32_t bsp_uart_rx_generation(void) { return generation; }
bool bsp_uart_receive(BspUartPacket *out) { if(!packet_ready)return false;*out=packet;packet_ready=false;return true; }
bool bsp_can_send_latest(BspCanTx *slot,const BspCanFrame *frame,volatile CanTxDebug *debug)
{ (void)slot;(void)debug;sent=*frame;sends++;return tx_ok; }
static void receive_remote(uint32_t time) {
    memset(&packet,0,sizeof packet);
    uint64_t bits=(uint64_t)1024 | (uint64_t)1024<<11 | (uint64_t)1024<<22 |
        (uint64_t)1024<<33 | (uint64_t)3<<44 | (uint64_t)1<<46;
    for(unsigned i=0;i<6;i++)packet.data[i]=(uint8_t)(bits>>(i*8));
    packet.data[17]=4;packet.length=18;packet.received_at_ms=time;packet_ready=true;
}
int main(void) {
    assert(remote_init());RemoteState rc, observed;
    assert(!remote_get_state(&observed) && observed.age_ms==UINT32_MAX);
    assert(!remote_get_state(NULL));
    now=10;remote_update(100,&rc);assert(!rc.online);
    receive_remote(now);remote_update(100,&rc);assert(rc.online && rc.channels.sw2==3);
    now=109;
    assert(remote_get_state(&observed) && observed.age_ms==99);
    now=110;
    assert(!remote_get_state(&observed) && observed.age_ms==100); /* Publisher paused. */
    receive_remote(now);
    assert(!remote_get_state(&observed) && packet_ready); /* Reader does not steal packets. */
    packet_ready=false;
    now=109;remote_update(100,&rc);assert(rc.online && rc.age_ms==99);
    now=110;remote_update(100,&rc);assert(!rc.online);
    receive_remote(now);remote_update(100,&rc);assert(rc.online);
    restart=true;remote_update(100,&rc);assert(!rc.online);
    assert(!remote_get_state(&observed)); /* Recovery invalidates prior channels. */
    receive_remote(now);remote_update(100,&rc);assert(rc.online);
    receive_remote(now);packet.length=17;remote_update(100,&rc);assert(!rc.online);
    now=UINT32_MAX-5;receive_remote(now);remote_update(100,&rc);assert(rc.online);
    now=4;remote_update(100,&rc);assert(rc.online && rc.age_ms==10);
    assert(remote_get_state(&observed) && observed.age_ms==10);
    rx_ok=false;remote_update(100,&rc);assert(!rc.online);


    const DjiMotorConfig cfg[]={
        {MOTOR_CHASSIS_FL,DJI_M3508,1,1,2000,100,20},
        {MOTOR_CHASSIS_FR,DJI_M3508,1,2,2000,100,20},
        {MOTOR_GIMBAL_YAW,DJI_GM6020,2,3,0,100,20}};
    DjiMotorHandle m[3];
    for(unsigned i=0;i<3;i++){m[i]=dji_motor_register(&cfg[i]);assert(m[i]);}
    assert(dji_motor_find(MOTOR_CHASSIS_FL)==m[0]);
    assert(!dji_motor_find(MOTOR_TRIGGER) && !dji_motor_find((DjiMotorId)-1));
    assert(!dji_motor_register(&cfg[0]));
    DjiMotorConfig bad=cfg[0];bad.id=MOTOR_TRIGGER;
    assert(!dji_motor_register(&bad)); /* Same feedback ID. */
    bad.esc_id=9;assert(!dji_motor_register(&bad));
    bad.esc_id=3;bad.output_limit_raw=20000;assert(!dji_motor_register(&bad));
    bad.output_limit_raw=1000;bad.command_timeout_ms=0;assert(!dji_motor_register(&bad));
    assert(!dji_motor_register(NULL));
    DjiMotorFeedback f[3];
    assert(dji_motor_read_many(m,3,f) && !f[0].received);
    const uint8_t data[8]={0x12,0x34,0xff,0x9c,0,5,30,0};
    now=50;
    assert(dji_motor_receive(1,0x201,data,8,now));
    assert(dji_motor_receive(1,0x202,data,8,now));
    assert(dji_motor_receive(2,0x207,data,8,now));
    assert(!dji_motor_receive(2,0x201,data,8,now));
    assert(dji_motor_receive(1,0x201,data,7,70));
    assert(dji_motor_read_many(m,3,f) && f[0].received_at_ms==50 && f[0].online);
    assert(f[0].measure.speed_rpm==-100);
    f[0].measure.speed_rpm=999;
    assert(dji_motor_read(m[0],f) && f[0].measure.speed_rpm==-100);
    assert(!dji_motor_read(NULL,f) && !f[0].received);
    assert(!dji_motor_read(m[0],NULL));
    const int16_t outputs[2]={3000,-3000};
    assert(dji_motor_set_outputs(m,outputs,2));
    assert(!dji_motor_set_output(m[2],1)); /* Read-only motor. */
    unsigned before=sends;
    dji_motor_service();
    assert(sends==before+1 && sent.bus==1 && sent.id==0x200);
    assert(sent.data[0]==7 && sent.data[1]==0xd0 && sent.data[2]==0xf8 && sent.data[3]==0x30);
    assert(debug_motor[MOTOR_CHASSIS_FL].queued_output_raw==2000);
    assert(debug_motor[MOTOR_CHASSIS_FR].queued_output_raw==-2000);
    assert(debug_motor[MOTOR_CHASSIS_FL].output_queue_count==1);
    for(unsigned i=4;i<8;i++)assert(sent.data[i]==0);
    DjiMotorHandle duplicate[2]={m[0],m[0]};
    assert(!dji_motor_set_outputs(duplicate,outputs,2));
    now=70;dji_motor_service(); /* Exact command deadline. */
    assert(debug_motor[MOTOR_CHASSIS_FL].queued_output_raw==0);
    for(unsigned i=0;i<8;i++)assert(sent.data[i]==0);
    now=71;assert(dji_motor_receive(1,0x201,data,8,now));
    dji_motor_service();assert(sent.data[0]==0 && sent.data[1]==0); /* Does not revive old command. */
    assert(dji_motor_set_output(m[0],123));
    dji_motor_service();assert(sent.data[1]==123);
    assert(dji_motor_relax(m[0]));dji_motor_service();assert(sent.data[1]==0);
    now=171;assert(dji_motor_read(m[0],f) && !f[0].online);
    assert(dji_motor_set_output(m[0],123));dji_motor_service();assert(sent.data[1]==0);
    now=UINT32_MAX-5;
    assert(dji_motor_receive(1,0x201,data,8,now));
    assert(dji_motor_set_output(m[0],123));
    now=4;dji_motor_service();assert(sent.data[1]==123);
    now=14;dji_motor_service();assert(sent.data[1]==0);
    now=20;assert(dji_motor_receive(1,0x201,data,8,now));
    assert(dji_motor_set_output(m[0],123));
    unsigned accepted=debug_motor[MOTOR_CHASSIS_FL].output_queue_count;
    tx_ok=false;dji_motor_service();
    assert(debug_motor[MOTOR_CHASSIS_FL].output_queue_count==accepted);
    assert(debug_motor[MOTOR_CHASSIS_FL].queued_output_raw==0);
    assert(dji_motor_read(m[0],f) && !f[0].transport_ready);
    tx_ok=true;dji_motor_service();assert(sent.data[1]==0);
    bus_ready=false;assert(dji_motor_set_output(m[0],123));dji_motor_service();
    assert(sent.data[1]==0);
    bus_ready=true;dji_motor_service();assert(sent.data[1]==0);
    /* Register different-bus duplicate ID and cross-type conflicting send slot. */
    bad=cfg[0];bad.id=MOTOR_TRIGGER;bad.bus=2;
    assert(dji_motor_register(&bad));
    bad.id=MOTOR_FRICTION_L;bad.type=DJI_M3508;bad.esc_id=7;
    assert(!dji_motor_register(&bad)); /* 0x207 collides with GM6020 feedback. */
    bad.esc_id=5;assert(dji_motor_register(&bad));
    bad.id=MOTOR_FRICTION_R;bad.type=DJI_GM6020;bad.esc_id=1;
    assert(!dji_motor_register(&bad)); /* 0x205 and 0x1ff/slot0 collide. */
    uint8_t near_wrap[8]={0x1f,0xfe,0,0,0,0,30,0};
    uint8_t after_wrap[8]={0,2,0,0,0,0,30,0};
    now=200;
    assert(dji_motor_receive(1,0x201,near_wrap,8,now));
    assert(dji_motor_read(m[0],f));uint32_t epoch=f[0].generation;
    assert(f[0].total_angle_deg==0);
    now=201;assert(dji_motor_receive(1,0x201,after_wrap,8,now));
    assert(dji_motor_read(m[0],f));
    assert(f[0].total_angle_deg>0.17f && f[0].total_angle_deg<0.18f);
    now=301;assert(dji_motor_receive(1,0x201,after_wrap,8,now));
    assert(dji_motor_read(m[0],f)&&f[0].generation==epoch+1&&f[0].total_angle_deg==0);
    puts("PASS: handles, duplicate IDs, snapshots, batching, limits, timeout, wrap, TX failure, read-only, multi-turn reset");
}
