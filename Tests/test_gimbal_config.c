#include "gimbal/gimbal_control.h"
#include "gimbal/gimbal_config.h"
#include <assert.h>
#include <stdio.h>

/* 配置用不同哨兵值编译，检查实际控制对象，而不是配置文件和自己比较。 */
static void check_pid(const pid_obj_t *pid, float kp, float ki, float kd,
                      float integral, float maximum)
{
    assert(pid->Kp == kp && pid->Ki == ki && pid->Kd == kd);
    assert(pid->IntegralLimit == integral && pid->MaxOut == maximum);
}
#define CHECK_AXIS(g, axis, i, mode) do { \
    check_pid(&(g)->angle[i], axis##_KP_A_##mode, axis##_KI_A_##mode, \
              axis##_KD_A_##mode, axis##_INTEGRAL_A_##mode, axis##_MAX_A_##mode); \
    check_pid(&(g)->speed[i], axis##_KP_V_##mode, axis##_KI_V_##mode, \
              axis##_KD_V_##mode, axis##_INTEGRAL_V_##mode, axis##_MAX_V_##mode); \
} while (0)

int main(void)
{
    GimbalController g;
    GimbalOutput out;
    GimbalCommand cmd = {.enabled=true, .neutral=true};
    GimbalInput in = {.valid=true,.relative_deg={0,0}};
    gimbal_control_reset(&g);
    CHECK_AXIS(&g, YAW, 0, IMU);
    CHECK_AXIS(&g, PITCH, 1, IMU);
    /* 完成归中，然后经历手动->自瞄->手动，确认切换确实装入对应参数。 */
    for (unsigned i=0; i<300; ++i)
        gimbal_control_update(&g, &cmd, &in, true, 0.002f, &out);
    assert(out.mode == GIMBAL_GYRO);
    cmd.auto_aim=true;
    gimbal_control_update(&g, &cmd, &in, true, 0.002f, &out);
    assert(out.mode == GIMBAL_AUTO);
    CHECK_AXIS(&g, YAW, 0, AUTO);
    CHECK_AXIS(&g, PITCH, 1, AUTO);
    cmd.auto_aim=false;
    gimbal_control_update(&g, &cmd, &in, true, 0.002f, &out);
    assert(out.mode == GIMBAL_GYRO);
    CHECK_AXIS(&g, YAW, 0, IMU);
    CHECK_AXIS(&g, PITCH, 1, IMU);
    puts("PASS: both gimbal axes use all five PID settings across reset and mode transitions");
}
