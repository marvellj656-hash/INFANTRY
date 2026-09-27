#include "transmission/transmission_protocol.h"
#include "transmission/transmission_service.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Windows 的标准 assert 可能弹窗等待点击；测试失败直接退出并打印行号。 */
#undef assert
#define assert(condition) do { if (!(condition)) { fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

static uint32_t fake_now;
uint32_t bsp_time_ms(void) { return fake_now; }
static unsigned frame_count;
static TransmissionFrame received;
static void capture(const TransmissionFrame *frame, void *context)
{
    (void)context;
    received = *frame;
    frame_count++;
}
/* 测试里的校验写成独立的逐项规则，不能用被测打包函数给自己作证。 */
static void fix_checksum(uint8_t *frame)
{
    unsigned sum = frame[0] + frame[1] + frame[2] + frame[3];
    unsigned add = sum;
    for (unsigned i = 0; i < frame[3]; ++i)
    {
        sum += frame[4u + i];
        add += sum;
    }
    frame[4u + frame[3]] = (uint8_t)sum;
    frame[5u + frame[3]] = (uint8_t)add;
}
static void target_frame(uint8_t *out)
{
    memset(out, 0, TRANSMISSION_RPY_FRAME_SIZE);
    const uint8_t prefix[] = {0xff, 4, 0x20, 25, 1,
        0xd2, 0x04, 0, 0, /* +1.234 deg -> MCU yaw=-1.234 */
        0x38, 0xff, 0xff, 0xff, /* -0.200 deg */
        0xe8, 0x03, 0, 0}; /* fire advice = 1 */
    memcpy(out, prefix, sizeof(prefix));
    fix_checksum(out);
}
static void feed(const uint8_t *bytes, size_t length, uint32_t time)
{
    fake_now = time;
    transmission_service_receive(bytes, length, time, time);
}


static void test_motion_vectors(void)
{
    uint8_t out[TRANSMISSION_RPY_FRAME_SIZE];
    const uint8_t expected[] = {
        0xff, 0x01, 0x20, 0x19, 0x00, 0x24, 0xfa, 0xff, 0xff,
        0xca, 0x08, 0, 0, 0, 0, 0, 0, 0xe8, 0x03, 0, 0,
        0x94, 0x1d, 0xfe, 0xff, /* -123500: -123.500 deg/s */
        0xc2, 0xb0, 0, 0,       /* +45250: +45.250 deg/s */
        0x32, 0x48};
    assert(transmission_encode_motion(out, sizeof(out), -1.5f, 2.25f,
        -123.5f, 45.25f, TRANSMISSION_TEAM_RED) == sizeof(expected));
    assert(memcmp(out, expected, sizeof(expected)) == 0);
    assert(transmission_checksum_valid(out, sizeof(out)));
    assert(!transmission_encode_motion(out, sizeof(out), 0, 0, NAN, 0, TRANSMISSION_TEAM_RED));
    assert(!transmission_encode_motion(out, sizeof(out), 0, 0, 0, INFINITY, TRANSMISSION_TEAM_RED));
    assert(!transmission_encode_motion(out, sizeof(out), INFINITY, 0, 0, 0, TRANSMISSION_TEAM_RED));
    assert(!transmission_encode_motion(out, sizeof(out), 0, NAN, 0, 0, TRANSMISSION_TEAM_RED));
    assert(!transmission_encode_motion(out, sizeof(out), 0, 181, 0, 0, TRANSMISSION_TEAM_RED));
    assert(!transmission_encode_motion(out, sizeof(out), 2000001, 0, 0, 0, TRANSMISSION_TEAM_RED));
    assert(!transmission_encode_motion(out, sizeof(out), 0, 0, 2000001, 0, TRANSMISSION_TEAM_RED));
    assert(!transmission_encode_motion(out, sizeof(out), 0, 0, 0, -2000001, TRANSMISSION_TEAM_RED));
    assert(!transmission_encode_motion(out, sizeof(out), 0, 0, 0, 0, (TransmissionTeam)2));
    assert(!transmission_encode_motion(out, sizeof(out)-1u, 0, 0, 0, 0, TRANSMISSION_TEAM_RED));
    assert(!transmission_encode_motion(NULL, sizeof(out), 0, 0, 0, 0, TRANSMISSION_TEAM_RED));
    assert(transmission_encode_motion(out, sizeof(out), -2000000, -180, 2000000,
        -2000000, TRANSMISSION_TEAM_UNKNOWN) == sizeof(out));
    const uint8_t expected_heartbeat[] = {0xff, 1, 0xf0, 1, 42, 0x1b, 0x0c};
    assert(transmission_encode_heartbeat(out, sizeof(out), 42) == sizeof(expected_heartbeat));
    assert(memcmp(out, expected_heartbeat, sizeof(expected_heartbeat)) == 0);
    assert(!transmission_encode_heartbeat(out, 6u, 42));
    assert(!transmission_encode_heartbeat(NULL, 7u, 42));
}

static void test_heartbeat_freshness(void)
{
    TransmissionState state;
    uint8_t target[TRANSMISSION_RPY_FRAME_SIZE];
    target_frame(target);
    uint8_t heartbeat[7] = {0xff, 4, 0xf0, 1, 42, 0x1e, 0x12};
    transmission_init();
    fake_now = 100u;
    transmission_service_update(fake_now, 1u, true);
    assert(!transmission_get_state(&state));
    assert(!state.heartbeat_online && state.heartbeat_age_ms == UINT32_MAX);
    feed(heartbeat, sizeof(heartbeat), 100u);
    assert(!transmission_get_state(&state));
    assert(state.online && state.heartbeat_online && state.heartbeat_at_ms == 100u);
    assert(state.heartbeat_age_ms == 0u && state.heartbeat_frames == 1u);
    assert(debug_transmission.heartbeat_frames == 1u && debug_transmission.heartbeat_online);
    feed(target, sizeof(target), 590u);
    fake_now = 600u; /* transmission任务没有再运行，getter仍撤销过期心跳。 */
    assert(transmission_get_state(&state));
    assert(state.online && !state.heartbeat_online && state.heartbeat_age_ms == 500u);
    /* 不停的目标包可令online=1，但不能谎报“还在收到F0心跳”。 */
    feed(target, sizeof(target), 601u);
    assert(transmission_get_state(&state) && !state.heartbeat_online);
    assert(debug_transmission.heartbeat_age_ms == 501u && !debug_transmission.heartbeat_online);

    uint8_t padded[TRANSMISSION_RPY_FRAME_SIZE] = {0xff, 4, 0xf0, 25, 43};
    fix_checksum(padded);
    feed(padded, sizeof(padded), 701u);
    assert(!transmission_get_state(&state)); /* 心跳不能挽救100ms旧目标。 */
    assert(state.heartbeat_online && state.heartbeat == 43u && state.heartbeat_frames == 2u);
    assert(!state.fire_advice);
    transmission_invalidate_target();
    assert(!transmission_get_state(&state) && state.heartbeat_online);
    transmission_service_update(702u, 2u, true);
    fake_now = 702u;
    assert(!transmission_get_state(&state));
    assert(!state.heartbeat_online && state.heartbeat_age_ms == UINT32_MAX && state.heartbeat == 0u);
    assert(state.heartbeat_frames == 2u); /* 历史统计保留，旧的心跳状态清除。 */
    feed(heartbeat, sizeof(heartbeat), 703u);
    assert(!transmission_get_state(&state) && state.heartbeat_online);
    transmission_service_update(704u, 2u, false);
    fake_now = 704u;
    assert(!transmission_get_state(&state) && !state.heartbeat_online && !state.online);
    transmission_service_update(705u, 2u, true);
    fake_now = 705u;
    assert(!transmission_get_state(&state) && !state.heartbeat_online);

    /* 短心跳同样支持时间绕回，500ms界限仍然严格。 */
    fake_now = UINT32_MAX - 30u;
    transmission_service_update(fake_now, 3u, true);
    feed(heartbeat, sizeof(heartbeat), fake_now);
    fake_now = 20u;
    assert(!transmission_get_state(&state) && state.heartbeat_online && state.heartbeat_age_ms == 51u);
    fake_now = 469u;
    assert(!transmission_get_state(&state) && !state.heartbeat_online && state.heartbeat_age_ms == 500u);
}

static bool send_busy;
static bool accept_send(const uint8_t *data, size_t length, void *context)
{
    (void)context;
    assert(transmission_checksum_valid(data, length));
    return !send_busy;
}
static void test_send_debug(void)
{
    transmission_init();
    TransmissionMotion motion = {.yaw_deg=-1.5f, .pitch_deg=2.25f,
        .yaw_rate_deg_s=-123.5f, .pitch_rate_deg_s=45.25f, .team=TRANSMISSION_TEAM_RED};
    transmission_service_send(0u, true, NULL, accept_send, NULL);
    assert(debug_transmission.tx_heartbeat_frames == 1u && debug_transmission.tx_frames == 0u);
    assert(debug_transmission.tx_ready && !debug_transmission.tx_reference_valid);
    assert(debug_transmission.tx_attitude_age_ms == UINT32_MAX);
    transmission_service_send(2u, true, &motion, accept_send, NULL);
    assert(debug_transmission.tx_frames == 1u && debug_transmission.tx_yaw_deg == -1.5f);
    assert(debug_transmission.tx_reference_valid && debug_transmission.tx_attitude_age_ms == 0u);
    assert(debug_transmission.tx_yaw_rate_deg_s == -123.5f);
    assert(debug_transmission.tx_pitch_rate_deg_s == 45.25f);
    send_busy = true;
    motion.yaw_deg = -2.0f;
    transmission_service_send(12u, true, &motion, accept_send, NULL);
    assert(debug_transmission.tx_busy == 1u && debug_transmission.tx_yaw_deg == -1.5f);
    assert(debug_transmission.tx_attitude_age_ms == 10u);
    transmission_service_send(100u, true, &motion, accept_send, NULL);
    assert(debug_transmission.tx_heartbeat_busy == 1u && debug_transmission.tx_heartbeat_frames == 1u);
    send_busy = false;
    transmission_service_send(102u, true, &motion, accept_send, NULL);
    assert(debug_transmission.tx_heartbeat == 1u && debug_transmission.tx_heartbeat_frames == 2u);
    transmission_service_send(104u, true, &motion, accept_send, NULL);
    assert(debug_transmission.tx_yaw_deg == -2.0f && debug_transmission.tx_frames == 2u);
    transmission_service_send(106u, true, NULL, accept_send, NULL);
    assert(!debug_transmission.tx_reference_valid && debug_transmission.tx_attitude_age_ms == UINT32_MAX);
    assert(debug_transmission.tx_yaw_deg == -2.0f);
    transmission_service_send(108u, false, &motion, accept_send, NULL);
    assert(!debug_transmission.tx_ready && !debug_transmission.tx_reference_valid);
    assert(debug_transmission.tx_attitude_age_ms == UINT32_MAX);
    transmission_service_send(110u, true, &motion, accept_send, NULL);
    transmission_service_send(112u, true, &motion, accept_send, NULL);
    assert(debug_transmission.tx_reference_valid && debug_transmission.tx_attitude_age_ms == 0u);
    transmission_service_update(114u, 123u, true); /* 任务未看到ready=false的快速重连。 */
    assert(!debug_transmission.tx_ready && debug_transmission.tx_attitude_age_ms == UINT32_MAX);
    unsigned heartbeats = debug_transmission.tx_heartbeat_frames;
    transmission_service_send(114u, true, &motion, accept_send, NULL);
    assert(debug_transmission.tx_heartbeat_frames == heartbeats + 1u);
    assert(debug_transmission.tx_attitude_age_ms == UINT32_MAX);
}

int main(void)
{
    uint8_t frame[TRANSMISSION_RPY_FRAME_SIZE];
    target_frame(frame);
    TransmissionParser parser;
    transmission_parser_reset(&parser);
    for (size_t i = 0; i < sizeof(frame); ++i)
        transmission_parser_feed(&parser, frame + i, 1u, capture, NULL);
    assert(frame_count == 1u && parser.frames == 1u);
    TransmissionTarget target;
    assert(transmission_decode_target(&received, &target));
    assert(fabsf(target.yaw_deg + 1.234f) < 0.0001f);
    assert(fabsf(target.pitch_deg + 0.2f) < 0.0001f);
    assert(target.fire_advice && target.raw_target_flag == 1u);

    /* 噪声、错长度、错校验后紧跟两个好包，必须重新找到边界。 */
    uint8_t noise[] = {0, 0x17, 0xff, 3, 0x20, 255};
    transmission_parser_feed(&parser, noise, sizeof(noise), capture, NULL);
    uint8_t corrupt[TRANSMISSION_RPY_FRAME_SIZE];
    memcpy(corrupt, frame, sizeof(frame));
    corrupt[5] ^= 1u;
    transmission_parser_feed(&parser, corrupt, sizeof(corrupt), capture, NULL);
    uint8_t joined[2u * TRANSMISSION_RPY_FRAME_SIZE];
    memcpy(joined, frame, sizeof(frame));
    memcpy(joined + sizeof(frame), frame, sizeof(frame));
    transmission_parser_feed(&parser, joined, sizeof(joined), capture, NULL);
    assert(frame_count == 3u && parser.checksum_errors && parser.length_errors);

    uint8_t packed[TRANSMISSION_RPY_FRAME_SIZE];
    assert(transmission_encode_attitude(packed, sizeof(packed), -1.5f, 2.25f,
                                       TRANSMISSION_TEAM_RED) == sizeof(packed));
    assert(packed[0] == 0xff && packed[1] == 1 && packed[2] == 0x20 && packed[3] == 25);
    assert(packed[5] == 0x24 && packed[6] == 0xfa && packed[7] == 0xff && packed[8] == 0xff);
    assert(packed[9] == 0xca && packed[10] == 0x08 && packed[11] == 0 && packed[12] == 0);
    assert(packed[17] == 0xe8 && packed[18] == 3);
    /* 已人工按旧SC/AC规则算出的固定向量，不只做encode/decode回环。 */
    assert(packed[29] == 0x12 && packed[30] == 0xd6);
    assert(transmission_checksum_valid(packed, sizeof(packed)));
    assert(!transmission_encode_attitude(packed, sizeof(packed), NAN, 0, TRANSMISSION_TEAM_RED));
    assert(!transmission_encode_attitude(packed, 4u, 0, 0, TRANSMISSION_TEAM_RED));

    transmission_init();
    fake_now = 100u;
    transmission_service_update(fake_now, 1u, true);
    feed(frame, sizeof(frame), 100u);
    TransmissionState state;
    assert(transmission_get_state(&state) && state.fire_advice && state.command_generation == 1u);
    /* 只收到心跳，不能刷新100ms目标期限；读者自身也必须检查期限。 */
    uint8_t heartbeat[7] = {0xff, 4, 0xf0, 1, 42, 0, 0};
    fix_checksum(heartbeat);
    feed(heartbeat, sizeof(heartbeat), 199u);
    assert(transmission_get_state(&state) && state.heartbeat == 42);
    fake_now = 200u;
    assert(!transmission_get_state(&state) && !state.fire_advice && state.online);
    transmission_service_update(fake_now, 1u, true);
    feed(heartbeat, sizeof(heartbeat), 201u);
    assert(!transmission_get_state(&state));
    assert(state.online && !state.fire_advice);

    /* 重连必须等新目标：心跳恢复或USB重新ready都不能复活旧目标。 */
    feed(frame, sizeof(frame), 210u);
    assert(transmission_get_state(&state));
    transmission_service_update(211u, 2u, true);
    fake_now = 211u;
    assert(!transmission_get_state(&state));
    feed(heartbeat, sizeof(heartbeat), 212u);
    assert(!transmission_get_state(&state));
    feed(frame, sizeof(frame), 213u);
    assert(transmission_get_state(&state));
    transmission_invalidate_target();
    assert(!transmission_get_state(&state));

    /* 半包跨越连接不能拼接。队列里100ms以前的完整包也不能变“新”。 */
    feed(frame, 15u, 220u);
    transmission_service_update(221u, 3u, true);
    feed(frame + 15u, sizeof(frame) - 15u, 222u);
    assert(!transmission_get_state(&state));
    fake_now = 500u;
    transmission_service_receive(frame, sizeof(frame), 300u, 500u);
    assert(!transmission_get_state(&state));

    /* 半包总用时超过100ms，即使每两块间隔都小于100ms，也不能组出新目标。 */
    feed(frame, 10u, 600u);
    feed(frame + 10u, 10u, 660u);
    feed(frame + 20u, sizeof(frame) - 20u, 720u);
    assert(!transmission_get_state(&state));

    /* 未知flag只原样保留；默认不因地址1/未知flag而改变旧协议兼容性。 */
    target_frame(frame);
    frame[1] = 1u;
    frame[4] = 7u;
    fix_checksum(frame);
    feed(frame, sizeof(frame), 510u);
    assert(transmission_get_state(&state) && state.raw_target_flag == 7u && state.destination == 1u);
    frame[13] = 1u; /* fire字段变成非0/1000：拒绝，并撤销之前目标 */
    fix_checksum(frame);
    feed(frame, sizeof(frame), 511u);
    assert(!transmission_get_state(&state));
    target_frame(frame);
    frame[9] = 0xff; frame[10] = 0xff; frame[11] = 0xff; frame[12] = 0x7f;
    fix_checksum(frame);
    feed(frame, sizeof(frame), 512u);
    assert(!transmission_get_state(&state));

    /* unsigned时间差支持系统毫秒计数绕回。 */
    target_frame(frame);
    fake_now = UINT32_MAX - 30u;
    transmission_service_update(fake_now, 4u, true);
    feed(frame, sizeof(frame), fake_now);
    fake_now = 20u;
    assert(transmission_get_state(&state) && state.age_ms == 51u);
    fake_now = 70u;
    assert(!transmission_get_state(&state));
    test_motion_vectors();
    test_heartbeat_freshness();
    test_send_debug();
    puts("transmission: legacy/motion vectors, stream, independent heartbeat, timeout and generations passed");
    return 0;
}
