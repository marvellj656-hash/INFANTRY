#include "cmd/cmd_mapping.h"
#include "motor/dji_motor_service.h"
#include "shoot/shoot_control.h"
#include "shoot/shoot_config.h"
#include "bsp_can.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Real command mapping -> real controller -> real motor grouping.
 * Only clock and the CAN transport are replaced by a recording stub. */
static uint32_t now;
static BspCanFrame sent;
static DjiMotorHandle motors[3];
static ShootController controller;
static CmdMapper mapper;
static CmdInput input;
static CmdOutput command;
static ShootOutput output;
uint32_t bsp_time_ms(void) { return now; }
bool bsp_can_is_ready(uint8_t bus) { return bus==2; }
bool bsp_can_send_latest(BspCanTx *slot,const BspCanFrame *frame,volatile CanTxDebug *debug)
{ (void)slot; debug->queued++; sent=*frame; return true; }

static void step(bool spinning)
{
    now+=2;
    input.remote.received_at_ms=now;
    cmd_map_update(&mapper,&input,&command);
    ShootInput feedback={.valid=true};
    for(unsigned i=0;i<3;i++) {
        int16_t rpm=spinning && i<2 ? (i==0 ? FRICTION_LEFT_DIRECTION*FRICTION_RPM : FRICTION_RIGHT_DIRECTION*FRICTION_RPM) : 0;
        uint16_t bits=(uint16_t)rpm;
        uint8_t bytes[8]={0,0,(uint8_t)(bits>>8),(uint8_t)bits,0,0,30,0};
        const uint16_t ids[3]={0x200+FRICTION_LEFT_ESC_ID,0x200+FRICTION_RIGHT_ESC_ID,0x203};
        assert(dji_motor_receive(2,ids[i],bytes,8,now));
        feedback.rpm[i]=rpm;
    }
    shoot_control_update(&controller,&command.shoot,&feedback,true,0.002f,&output);
    assert(dji_motor_set_outputs(motors,output.raw,3));
    dji_motor_service();
    assert(sent.bus==2 && sent.id==0x200 && sent.length==8);
}
static void expect_zero(void)
{
    for(unsigned i=0;i<3;i++) assert(output.raw[i]==0);
    for(unsigned i=0;i<8;i++) assert(sent.data[i]==0);
    assert(debug_motor[MOTOR_FRICTION_L].queued_output_raw==0);
    assert(debug_motor[MOTOR_FRICTION_R].queued_output_raw==0);
    assert(debug_motor[MOTOR_TRIGGER].queued_output_raw==0);
}
static int signed_slot(unsigned slot) {
    unsigned value=((unsigned)sent.data[slot*2]<<8)|sent.data[slot*2+1];
    return value>=32768 ? (int)value-65536 : (int)value;
}
static void expect_brake(void) {
    assert(output.braking && !output.friction_ready && output.raw[2]==0);
    assert(output.raw[0]*FRICTION_LEFT_DIRECTION<0 && output.raw[1]*FRICTION_RIGHT_DIRECTION<0);
    assert(signed_slot(FRICTION_LEFT_ESC_ID-1)==output.raw[0]);
    assert(signed_slot(FRICTION_RIGHT_ESC_ID-1)==output.raw[1]);
    assert(signed_slot(2)==0);
}
static void start(void)
{
    input.remote.online=true;
    input.remote.channels.sw1=RC_SWITCH_MIDDLE;
    input.remote.channels.sw2=RC_SWITCH_MIDDLE;
    input.remote.channels.wheel=0;
    for(unsigned i=0;i<5;i++) step(false);
    assert(output.armed && output.raw[0]*FRICTION_LEFT_DIRECTION>0 && output.raw[1]*FRICTION_RIGHT_DIRECTION>0);
}
int main(void)
{
    DjiMotorConfig config[3]={
        {MOTOR_FRICTION_L,DJI_M3508,2,FRICTION_LEFT_ESC_ID,16000,20,20},
        {MOTOR_FRICTION_R,DJI_M3508,2,FRICTION_RIGHT_ESC_ID,16000,20,20},
        {MOTOR_TRIGGER,DJI_M2006,2,3,10000,20,20}};
    for(unsigned i=0;i<3;i++) { motors[i]=dji_motor_register(&config[i]); assert(motors[i]); }
    shoot_control_reset(&controller);
    start();
    /* Normal OFF brakes; hard disable/loss always zero even while spinning. */
    input.remote.channels.sw1=RC_SWITCH_UP;
    step(true); assert(!command.shoot.friction); expect_brake();
    for(unsigned i=0;i<30;i++) step(false);
    expect_zero();
    start(); input.remote.channels.sw2=RC_SWITCH_UP;
    step(true); assert(!command.shoot.enabled); expect_zero();
    start(); input.remote.online=false;
    step(true); expect_zero();
    /* Early single-shot request is not saved for a delayed shot. */
    start(); input.remote.channels.wheel=660;
    for(unsigned i=0;i<400;i++) step(false);
    assert(output.armed && !output.friction_ready && output.raw[2]==0);
    for(unsigned i=0;i<60;i++) step(true);
    assert(output.friction_ready && output.raw[2]==0);
    input.remote.channels.wheel=0; step(true);
    input.remote.channels.wheel=660; step(true);
    assert(output.raw[2]>0 && debug_motor[MOTOR_TRIGGER].queued_output_raw>0);
    input.remote.channels.sw1=RC_SWITCH_UP; step(true); expect_brake();
    /* Disable also resets an active continuous shot. */
    start();
    for(unsigned i=0;i<400;i++) step(true);
    input.remote.channels.sw1=RC_SWITCH_DOWN;
    input.remote.channels.wheel=660; step(true);
    assert(output.friction_ready && output.raw[2]>0);
    input.remote.channels.sw2=RC_SWITCH_UP; step(true); expect_zero();
    puts("PASS: command -> shoot -> motor frame: OFF, disable, RC loss, spinning feedback, speed gate and single/continuous trigger");
}
