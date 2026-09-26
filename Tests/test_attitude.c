#include "attitude.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
int main(void) {
    AttitudeEstimator s;AttitudeResult out;attitude_reset(&s);
    float a[3]={0,0,9.80665f},g[3]={0.001f,-0.002f,0.003f};
    for(unsigned i=0;i<499;i++)assert(!attitude_update(&s,g,a,0.002f,&out));
    assert(attitude_update(&s,g,a,0.002f,&out));
    for(unsigned i=0;i<1000;i++)assert(attitude_update(&s,g,a,0.002f,&out));
    assert(fabsf(out.yaw_deg)<0.1f&&fabsf(out.pitch_deg)<0.1f);
    g[2]+=0.5f;
    for(unsigned i=0;i<1000;i++)assert(attitude_update(&s,g,a,0.002f,&out));
    assert(out.yaw_deg< -55&&out.yaw_deg> -60);
    assert(!attitude_update(&s,g,a,0.1f,&out)&&!out.valid&&!s.initialized);
    g[2]=0.003f;
    a[0]=-9.80665f*0.5f;a[2]=9.80665f*0.8660254f;
    for(unsigned i=0;i<600;i++)attitude_update(&s,g,a,0.002f,&out);
    assert(out.valid&&fabsf(out.pitch_deg-30)<0.5f);
    a[0]=NAN;assert(!attitude_update(&s,g,a,0.002f,&out));
    puts("PASS: EKF stationary bias, tilt initialization, yaw sign/integration, invalid dt, reset");
}
