#include "cmd/cmd_mapping.h"
#include "chassis/chassis_control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void near(float actual, float expected)
{
    assert(fabsf(actual - expected) < 0.001f);
}

static CmdInput manual_input(void)
{
    CmdInput in = {0};
    in.remote.online = true;
    in.remote.received_at_ms = 1234;
    in.remote.channels.sw1 = RC_SWITCH_UP;
    in.remote.channels.sw2 = RC_SWITCH_MIDDLE;
    return in;
}

static void expect_disabled(const CmdOutput *out)
{
    assert(!out->chassis.enabled && !out->gimbal.enabled && !out->shoot.enabled);
    assert(!out->spin && !out->shoot.friction && !out->shoot.fire);
    assert(!out->chassis.follow_gimbal);
    assert(!out->gimbal.auto_aim && out->shoot.mode == SHOOT_STOP);
    near(out->chassis.target.vx, 0);
    near(out->chassis.target.vy, 0);
    near(out->chassis.target.wz, 0);
    near(out->gimbal.yaw_rate_deg_s, 0);
    near(out->gimbal.pitch_rate_deg_s, 0);
}

static void test_switch_table(void)
{
    /* This table is the operator contract, including all nine physical combinations. */
    const struct {
        uint8_t left, right;
        bool enabled, spin, friction;
        ShootMode mode;
    } cases[] = {
        {1, 1, false, false, false, SHOOT_STOP},
        {3, 1, false, false, false, SHOOT_STOP},
        {2, 1, false, false, false, SHOOT_STOP},
        {1, 3, true, false, false, SHOOT_SINGLE},
        {3, 3, true, false, true, SHOOT_SINGLE},
        {2, 3, true, true, true, SHOOT_SINGLE},
        {1, 2, true, false, false, SHOOT_SINGLE},
        {3, 2, true, false, true, SHOOT_SINGLE},
        {2, 2, true, true, true, SHOOT_SINGLE}
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        CmdMapper mapper = {0};
        CmdInput in = manual_input();
        CmdOutput out;
        in.remote.channels.sw1 = cases[i].left;
        in.remote.channels.sw2 = cases[i].right;
        cmd_map_update(&mapper, &in, &out);
        assert(out.chassis.enabled == cases[i].enabled);
        assert(out.chassis.follow_gimbal == (cases[i].enabled && !cases[i].spin));
        assert(out.gimbal.enabled == cases[i].enabled);
        assert(out.shoot.enabled == cases[i].enabled);
        assert(out.spin == cases[i].spin && out.shoot.friction == cases[i].friction);
        assert(out.auto_requested == (cases[i].right==RC_SWITCH_DOWN));
        assert(!out.gimbal.auto_aim); /* no fresh target yet */
        assert(out.shoot.mode == cases[i].mode && !out.shoot.fire);
        assert(out.chassis.neutral && out.gimbal.neutral);
        assert(out.chassis.received_at_ms == 1234);
        assert(out.gimbal.received_at_ms == 1234 && out.shoot.received_at_ms == 1234);
        /* 同一组开关再提供新鲜目标：左上允许AUTO，但仍不能开摩擦。 */
        in.gimbal_fresh = true;
        in.gimbal.centered = in.gimbal.reference_valid = true;
        in.vision.online = in.vision.aim_valid = true;
        in.vision.fire_advice = true;
        cmd_map_update(&mapper, &in, &out);
        assert(out.gimbal.auto_aim ==
               (cases[i].enabled && cases[i].right == RC_SWITCH_DOWN));
        assert(out.shoot.friction == cases[i].friction && !out.shoot.fire);
        assert(out.spin == cases[i].spin);
        assert(out.chassis.follow_gimbal == (cases[i].enabled && !cases[i].spin));
    }
}

static void test_loss_and_invalid_switches(void)
{
    CmdMapper mapper = {0};
    CmdInput in = manual_input();
    CmdOutput out;
    in.remote.channels.sw1 = RC_SWITCH_DOWN;
    in.remote.channels.sw2 = RC_SWITCH_DOWN;
    in.remote.channels.ch[0] = 660;
    in.remote.channels.ch[2] = 660;
    in.remote.channels.wheel = 660;
    cmd_map_update(&mapper, &in, &out);
    assert(out.spin && out.shoot.fire && out.shoot.friction);
    /* Disconnection leaves old channel values in the snapshot: none may drive outputs. */
    in.remote.online = false;
    cmd_map_update(&mapper, &in, &out);
    expect_disabled(&out);
    in.remote.online = true;
    const uint8_t invalid[] = {0, 4, 255};
    for (unsigned i = 0; i < sizeof(invalid); ++i) {
        in.remote.channels.sw1 = invalid[i];
        in.remote.channels.sw2 = RC_SWITCH_MIDDLE;
        cmd_map_update(&mapper, &in, &out);
        expect_disabled(&out);
        in.remote.channels.sw1 = RC_SWITCH_MIDDLE;
        in.remote.channels.sw2 = invalid[i];
        cmd_map_update(&mapper, &in, &out);
        expect_disabled(&out);
    }
}

static void test_neutral_axes_and_spin_startup(void)
{
    CmdMapper mapper = {0};
    CmdInput in = manual_input();
    CmdOutput out;
    /* All four axes participate in rearming, including left vertical CH4. */
    for (unsigned axis = 0; axis < 4; ++axis) {
        for (int sign = -1; sign <= 1; sign += 2) {
            in.remote.channels.ch[axis] = (int16_t)(5 * sign);
            cmd_map_update(&mapper, &in, &out);
            assert(out.chassis.neutral && out.gimbal.neutral);
            in.remote.channels.ch[axis] = (int16_t)(6 * sign);
            cmd_map_update(&mapper, &in, &out);
            assert(!out.chassis.neutral && !out.gimbal.neutral);
            in.remote.channels.ch[axis] = 0;
        }
    }
    in.remote.channels.mouse_x = 1;
    cmd_map_update(&mapper, &in, &out);
    assert(!out.gimbal.neutral);
    in.remote.channels.mouse_x = 0;
    in.remote.channels.mouse_y = -1;
    cmd_map_update(&mapper, &in, &out);
    assert(!out.gimbal.neutral);
    in.remote.channels.mouse_y = 0;
    mapper.mouse_x_filtered=mapper.mouse_y_filtered=0; /* 分开测试满杆倍率和滤波尾巴。 */
    in.remote.channels.ch[2] = 660;
    in.remote.channels.ch[3] = -660;
    cmd_map_update(&mapper, &in, &out);
    near(out.chassis.target.vx, 0);
    near(out.chassis.target.vy, 0);
    near(out.chassis.target.wz, 0); /* CH3 cannot turn the chassis and gimbal together. */
    near(out.gimbal.yaw_rate_deg_s, 297);
    near(out.gimbal.pitch_rate_deg_s, -297);

    in.remote.channels.ch[2] = in.remote.channels.ch[3] = 0;
    in.remote.channels.sw1 = RC_SWITCH_DOWN;
    in.remote.channels.sw2 = RC_SWITCH_MIDDLE;
    cmd_map_update(&mapper, &in, &out);
    assert(out.spin && out.chassis.neutral && out.chassis.target.wz > 0);
    /* Exercise the actual chassis gate: a nonzero spin target must not prevent rearming. */
    ChassisController chassis;
    ChassisOutput result;
    const float rpm[4] = {0};
    chassis_control_reset(&chassis);
    chassis_control_update(&chassis, out.chassis.target, rpm, out.chassis.enabled,
                           out.chassis.neutral, 0.002f, 60, &result);
    assert(result.armed);
    chassis_control_update(&chassis, out.chassis.target, rpm, out.chassis.enabled,
                           out.chassis.neutral, 0.002f, 60, &result);
    assert(result.velocity.wz > 0);
    assert(result.target_rpm[0] != 0 || result.target_rpm[1] != 0);
}

static void test_trigger_and_keyboard(void)
{
    CmdMapper mapper = {0};
    CmdInput in = manual_input();
    CmdOutput out;
    in.remote.channels.sw1 = RC_SWITCH_MIDDLE;
    const struct { int16_t wheel; bool fire; ShootMode mode; } wheel_cases[] = {
        {0, false, SHOOT_SINGLE}, {199, false, SHOOT_SINGLE}, {200, true, SHOOT_SINGLE},
        {660, true, SHOOT_SINGLE}, {-199, false, SHOOT_SINGLE},
        {-200, true, SHOOT_REVERSE}, {-660, true, SHOOT_REVERSE}
    };
    for (unsigned i = 0; i < sizeof(wheel_cases) / sizeof(wheel_cases[0]); ++i) {
        in.remote.channels.wheel = wheel_cases[i].wheel;
        cmd_map_update(&mapper, &in, &out);
        assert(out.shoot.fire == wheel_cases[i].fire && out.shoot.mode == wheel_cases[i].mode);
    }
    in.remote.channels.wheel = 0;
    in.remote.channels.mouse_left = 1;
    cmd_map_update(&mapper, &in, &out);
    assert(out.shoot.fire);
    in.remote.channels.mouse_left = 0;
    in.vision.fire_advice = true;
    cmd_map_update(&mapper, &in, &out);
    assert(!out.shoot.fire); /* NUC advice does not manufacture an operator trigger. */

    in.remote.channels.keys = (1u << 10); /* G stops even when the physical switch requests on. */
    cmd_map_update(&mapper, &in, &out);
    assert(!out.shoot.friction);
    in.remote.channels.keys = 0;
    cmd_map_update(&mapper, &in, &out);
    assert(!out.shoot.friction);
    in.remote.channels.keys = (1u << 9) | (1u << 10); /* G wins over simultaneous F. */
    cmd_map_update(&mapper, &in, &out);
    assert(!out.shoot.friction);
    in.remote.channels.keys = 0;
    cmd_map_update(&mapper, &in, &out);
    in.remote.channels.keys = (1u << 9);
    cmd_map_update(&mapper, &in, &out);
    assert(out.shoot.friction);
    in.remote.channels.keys = (1u << 14); /* V acts on edges, not every 10 ms while held. */
    cmd_map_update(&mapper, &in, &out);
    assert(out.shoot.mode == SHOOT_CONTINUOUS);
    cmd_map_update(&mapper, &in, &out);
    assert(out.shoot.mode == SHOOT_CONTINUOUS);
    in.remote.channels.keys = 0;
    cmd_map_update(&mapper, &in, &out);
    in.remote.channels.keys = (1u << 14);
    cmd_map_update(&mapper, &in, &out);
    assert(out.shoot.mode == SHOOT_SINGLE);
    in.remote.channels.sw1 = RC_SWITCH_DOWN;
    cmd_map_update(&mapper, &in, &out);
    assert(out.spin && out.shoot.mode == SHOOT_SINGLE);
    in.remote.channels.keys = (1u << 10);
    cmd_map_update(&mapper, &in, &out);
    assert(!out.shoot.friction);
}

static void test_auto_reference_and_recovery(void)
{
    CmdMapper mapper = {0};
    CmdInput in = manual_input();
    CmdOutput out;
    in.remote.channels.mouse_right = 1;
    in.gimbal_fresh = true;
    in.gimbal.centered = in.gimbal.reference_valid = true;
    in.gimbal.reference_generation = 7;
    in.gimbal.target_deg[0] = 100;
    in.gimbal.target_deg[1] = 10;
    in.vision.aim_valid = in.vision.online = true;
    in.vision.link_generation = 2;
    in.vision.yaw_deg = 5;
    in.vision.pitch_deg = -2;
    in.vision.fire_advice = true;
    cmd_map_update(&mapper, &in, &out);
    assert(out.gimbal.auto_aim && !out.shoot.fire);
    near(out.gimbal.auto_yaw_deg, 105);
    near(out.gimbal.auto_pitch_deg, 8);
    for (unsigned i = 0; i < 50; ++i) {
        /* Feed the commanded target back as the task would: never add the same offset twice. */
        in.gimbal.target_deg[0] = out.gimbal.auto_yaw_deg;
        in.gimbal.target_deg[1] = out.gimbal.auto_pitch_deg;
        cmd_map_update(&mapper, &in, &out);
        near(out.gimbal.auto_yaw_deg, 105);
        near(out.gimbal.auto_pitch_deg, 8);
    }
    in.vision.command_generation++;
    in.vision.yaw_deg = -7;
    in.vision.pitch_deg = 4;
    cmd_map_update(&mapper, &in, &out);
    near(out.gimbal.auto_yaw_deg, 93);
    near(out.gimbal.auto_pitch_deg, 14);

    /* Freshness is validated by the transmission snapshot API before mapping. */
    in.vision.aim_valid = false;
    in.remote.channels.ch[2] = 660;
    cmd_map_update(&mapper, &in, &out);
    assert(!out.gimbal.auto_aim && !mapper.auto_active);
    near(out.gimbal.yaw_rate_deg_s, 297);
    in.vision.aim_valid = true;
    in.gimbal.target_deg[0] = 200;
    in.gimbal.target_deg[1] = 3;
    cmd_map_update(&mapper, &in, &out);
    near(out.gimbal.auto_yaw_deg, 193);
    near(out.gimbal.auto_pitch_deg, 7);

    in.vision.link_generation++;
    in.gimbal.target_deg[0] = 300;
    in.gimbal.target_deg[1] = 1;
    cmd_map_update(&mapper, &in, &out);
    near(out.gimbal.auto_yaw_deg, 293);
    near(out.gimbal.auto_pitch_deg, 5);
    in.gimbal.reference_generation++;
    in.gimbal.target_deg[0] = 400;
    cmd_map_update(&mapper, &in, &out);
    near(out.gimbal.auto_yaw_deg, 393);

    in.gimbal_fresh = false;
    cmd_map_update(&mapper, &in, &out);
    assert(!out.gimbal.auto_aim);
    in.gimbal_fresh = true;
    in.gimbal.centered = false;
    cmd_map_update(&mapper, &in, &out);
    assert(!out.gimbal.auto_aim);
    in.gimbal.centered = true;
    in.gimbal.reference_valid = false;
    cmd_map_update(&mapper, &in, &out);
    assert(!out.gimbal.auto_aim);
    in.gimbal.reference_valid = true;
    in.remote.channels.mouse_right = 0;
    cmd_map_update(&mapper, &in, &out);
    assert(!out.gimbal.auto_aim);
    in.remote.channels.mouse_right = 1;
    in.remote.online = false;
    cmd_map_update(&mapper, &in, &out);
    expect_disabled(&out);
}

static void test_switch_auto_and_spin(void)
{
    CmdMapper mapper={0};CmdInput in=manual_input();CmdOutput out;
    in.gimbal_fresh=true;
    in.gimbal.centered=in.gimbal.reference_valid=true;
    in.gimbal.target_deg[0]=100;in.gimbal.target_deg[1]=10;
    in.vision.online=in.vision.aim_valid=true;
    in.vision.yaw_deg=5;in.vision.pitch_deg=-2;
    in.vision.fire_advice=true;
    in.remote.channels.sw1=RC_SWITCH_MIDDLE;
    cmd_map_update(&mapper,&in,&out);
    assert(out.chassis.follow_gimbal && !out.gimbal.auto_aim && !out.spin);
    assert(out.auto_block_flags & AUTO_BLOCK_NOT_REQUESTED);
    in.remote.channels.sw2=RC_SWITCH_DOWN;
    cmd_map_update(&mapper,&in,&out);
    assert(out.auto_requested && out.gimbal.auto_aim && out.auto_block_flags==0);
    assert(out.chassis.follow_gimbal && !out.spin && !out.shoot.fire);
    near(out.gimbal.auto_yaw_deg,105);near(out.gimbal.auto_pitch_deg,8);
    for(unsigned i=0;i<30;i++) {
        in.gimbal.target_deg[0]=out.gimbal.auto_yaw_deg;
        cmd_map_update(&mapper,&in,&out);
        near(out.gimbal.auto_yaw_deg,105); /* no repeated accumulation */
    }
    in.remote.channels.sw1=RC_SWITCH_DOWN;
    cmd_map_update(&mapper,&in,&out);
    assert(out.spin && !out.chassis.follow_gimbal && out.gimbal.auto_aim);
    assert(out.shoot.mode==SHOOT_SINGLE && !out.shoot.fire);
    in.vision.aim_valid=false;in.remote.channels.ch[2]=100;
    cmd_map_update(&mapper,&in,&out);
    assert(!out.gimbal.auto_aim && out.auto_requested);
    assert(out.auto_block_flags & AUTO_BLOCK_TARGET_STALE);
    assert(out.gimbal.yaw_rate_deg_s>0); /* target timeout returns manual control */
    in.vision.aim_valid=true;in.vision.online=false;
    cmd_map_update(&mapper,&in,&out);
    assert(!out.gimbal.auto_aim && (out.auto_block_flags&AUTO_BLOCK_NUC_OFFLINE));
    in.vision.online=true;in.gimbal_fresh=false;
    cmd_map_update(&mapper,&in,&out);
    assert(!out.gimbal.auto_aim && (out.auto_block_flags&AUTO_BLOCK_GIMBAL_STALE));
    in.gimbal_fresh=true;in.gimbal.reference_valid=false;
    cmd_map_update(&mapper,&in,&out);
    assert(!out.gimbal.auto_aim && (out.auto_block_flags&AUTO_BLOCK_REFERENCE));
    in.remote.channels.sw2=RC_SWITCH_UP;
    cmd_map_update(&mapper,&in,&out);expect_disabled(&out);
    assert(out.auto_block_flags&AUTO_BLOCK_DISABLED);
}

int main(void)
{
    test_switch_auto_and_spin();
    test_switch_table();
    test_loss_and_invalid_switches();
    test_neutral_axes_and_spin_startup();
    test_trigger_and_keyboard();
    test_auto_reference_and_recovery();
    puts("PASS: nine switch combinations, offline gates, neutral/spin startup, axes, trigger, keyboard, AUTO reference/recovery");
    return 0;
}
