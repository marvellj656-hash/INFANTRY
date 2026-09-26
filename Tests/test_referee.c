#include "referee/referee_protocol.h"
#include "referee/referee.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t now_ms;
uint32_t bsp_time_ms(void) { return now_ms; }
static unsigned callbacks;
static uint16_t last_command;
static uint16_t last_length;
static void got_frame(const RefereeFrame *frame, void *context)
{
    (void)context;
    ++callbacks;
    last_command = frame->command;
    last_length = frame->length;
}

/* 固定向量独立于 encode/feed；其结果与旧工程 CRC 查表版本一致。 */
static void test_crc_vectors(void)
{
    static const uint8_t text[] = "123456789";
    assert(referee_crc8(text, 9u) == 0x0bu);
    assert(referee_crc16(text, 9u) == 0x6f91u);
}

static void test_stream(void)
{
    RefereeParser parser;
    referee_protocol_init(&parser);
    uint8_t payload[13] = {3u, 1u, 100u, 0u, 200u, 0u, 50u, 0u, 100u, 0u, 80u, 0u, 7u};
    uint8_t frame[128];
    size_t length = referee_protocol_encode(0x0201, 42u, payload, sizeof(payload), frame, sizeof(frame));
    assert(length == 22u);
    callbacks = 0;
    /* 无论串口在哪个字节分成两块，都应该还原一包。 */
    for (size_t cut = 0; cut <= length; ++cut) {
        referee_protocol_init(&parser);
        referee_protocol_feed(&parser, frame, cut, got_frame, NULL);
        referee_protocol_feed(&parser, frame + cut, length - cut, got_frame, NULL);
        assert(parser.valid_frames == 1u);
        assert(parser.used == 0u);
    }
    assert(callbacks == length + 1u);
    assert(last_command == 0x0201 && last_length == 13u);

    referee_protocol_init(&parser);
    uint8_t joined[48] = {0x11, 0x22, 0x33, 0x44};
    memcpy(joined + 4, frame, length);
    memcpy(joined + 4 + length, frame, length);
    referee_protocol_feed(&parser, joined, sizeof(joined), got_frame, NULL);
    assert(parser.valid_frames == 2u && parser.discarded_bytes == 4u);

    /* 假帧头后紧跟真帧头；错误恢复不能连真帧头一起丢掉。 */
    referee_protocol_init(&parser);
    uint8_t false_start[] = {0xa5, 0xff};
    referee_protocol_feed(&parser, false_start, sizeof(false_start), got_frame, NULL);
    referee_protocol_feed(&parser, frame, length, got_frame, NULL);
    assert(parser.header_errors != 0u && parser.valid_frames == 1u);

    referee_protocol_init(&parser);
    uint8_t corrupt[128];
    memcpy(corrupt, frame, length);
    corrupt[8] ^= 0x10u;
    referee_protocol_feed(&parser, corrupt, length, got_frame, NULL);
    referee_protocol_feed(&parser, frame, length, got_frame, NULL);
    assert(parser.crc_errors == 1u && parser.valid_frames == 1u);

    referee_protocol_init(&parser);
    uint8_t oversized[5] = {0xa5, 120u, 0u, 0u, 0u};
    oversized[4] = referee_crc8(oversized, 4u);
    referee_protocol_feed(&parser, oversized, sizeof(oversized), got_frame, NULL);
    referee_protocol_feed(&parser, frame, length, got_frame, NULL);
    assert(parser.length_errors == 1u && parser.valid_frames == 1u);

    /* 允许恰好 128 字节；129 字节一定拒绝。 */
    uint8_t maximum[120] = {0};
    length = referee_protocol_encode(0x0301, 0u, maximum, 119u, frame, sizeof(frame));
    assert(length == 128u);
    referee_protocol_init(&parser);
    referee_protocol_feed(&parser, frame, length, got_frame, NULL);
    assert(parser.valid_frames == 1u && last_length == 119u);
    assert(referee_protocol_encode(0x0301, 0u, maximum, 120u, frame, sizeof(frame)) == 0u);
    assert(referee_protocol_encode(1u, 0u, NULL, 1u, frame, sizeof(frame)) == 0u);
    assert(referee_protocol_encode(1u, 0u, maximum, 1u, frame, 9u) == 0u);

    /* CRC16 失败的外包里嵌有一整个合法包，逐字节恢复仍能找到它。 */
    uint8_t inner[32];
    size_t inner_length = referee_protocol_encode(0x0201, 4u, payload, 13u, inner, sizeof(inner));
    uint8_t outer_payload[40] = {0};
    memcpy(outer_payload + 2, inner, inner_length);
    length = referee_protocol_encode(0x9999, 0u, outer_payload, sizeof(outer_payload), frame, sizeof(frame));
    frame[length - 1] ^= 1u;
    referee_protocol_init(&parser);
    referee_protocol_feed(&parser, frame, length, got_frame, NULL);
    assert(parser.crc_errors == 1u && parser.valid_frames == 1u && last_command == 0x0201);
}

static void send_payload(uint16_t command, const uint8_t *payload, size_t length)
{
    uint8_t frame[128];
    size_t size = referee_protocol_encode(command, 0u, payload, length, frame, sizeof(frame));
    assert(size != 0u);
    referee_receive(frame, size, now_ms);
}

static void test_state(void)
{
    now_ms = 100u;
    referee_init();
    RefereeState state;
    assert(!referee_get_state(&state));
    assert(!state.status.received && state.status.age_ms == UINT32_MAX);
    uint8_t status[13] = {103u, 2u, 100u, 0u, 200u, 0u, 40u, 0u, 120u, 0u, 80u, 0u, 5u};
    send_payload(0x0201, status, sizeof(status));
    assert(referee_get_state(&state));
    assert(state.robot_id == 103u && state.status.fresh && !state.power_heat.fresh);
    uint32_t initial_generation = state.generation;
    assert(state.chassis_power_limit_w == 80u && state.shooter_heat_limit == 120u);
    assert(state.gimbal_power_enabled && !state.chassis_power_enabled && state.shooter_power_enabled);
    /* 调用者改自己的副本，不得改掉模块拥有的状态。 */
    state.robot_id = 9u;
    referee_get_state(&state);
    assert(state.robot_id == 103u);

    uint8_t power[16] = {0x34u, 0x12u, 0x78u, 0x56u, 0u, 0u, 0x20u, 0x42u,
                         50u, 0u, 60u, 0u, 70u, 0u, 80u, 0u};
    send_payload(0x0202, power, sizeof(power));
    referee_get_state(&state);
    assert(state.power_heat.fresh && state.buffer_energy_j == 50u);
    assert(state.shooter_heat_17mm_1 == 60u && state.shooter_heat_42mm == 80u);
    assert(state.legacy_current_raw == 0x5678u && state.legacy_power_bits == 0x42200000u);

    uint8_t bullet[] = {1u, 1u, 12u, 0u, 0u, 0xd4u, 0x41u}; /* 26.5 m/s */
    send_payload(0x0207, bullet, sizeof(bullet));
    referee_get_state(&state);
    assert(state.shooting.fresh && fabsf(state.bullet_speed_m_s - 26.5f) < 0.0001f);
    bullet[5] = 0xc0u; bullet[6] = 0x7fu; /* NaN 不可写入快照。 */
    send_payload(0x0207, bullet, sizeof(bullet));
    referee_get_state(&state);
    assert(state.bullet_speed_m_s == 26.5f && debug_referee.invalid_payloads == 1u);

    uint8_t allowance[] = {0x2cu, 1u, 10u, 0u, 20u, 0u};
    send_payload(0x0208, allowance, sizeof(allowance));
    referee_get_state(&state);
    assert(state.projectile_allowance_17mm == 300u && state.remaining_gold_coin == 20u);
    uint8_t game[] = {0x41u, 0x68u, 1u, 1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
    send_payload(0x0001, game, sizeof(game));
    referee_get_state(&state);
    assert(state.game_type == 1u && state.game_progress == 4u && state.stage_remaining_s == 360u);
    assert(state.sync_timestamp == UINT64_C(0x0807060504030201));
    uint8_t event[] = {1u, 2u, 3u, 4u};
    send_payload(0x0101, event, sizeof(event));
    referee_get_state(&state);
    assert(state.event_bits == UINT32_C(0x04030201));

    /* 合法 CRC 但业务长度错误，不得越界读取/覆盖有效状态。 */
    send_payload(0x0201, status, 12u);
    referee_get_state(&state);
    assert(state.robot_id == 103u && debug_referee.invalid_payloads == 2u);

    /* 陌生命令不能给功率热量“续命”。快照读取自己计算过期。 */
    now_ms = 700u;
    send_payload(0x8888, NULL, 0u);
    assert(referee_get_state(&state));
    assert(state.status.fresh && !state.power_heat.fresh);
    assert(debug_referee.unsupported_commands == 1u);
    now_ms = 1700u;
    assert(!referee_get_state(&state));
    assert(!state.status.fresh && !state.power_heat.fresh);

    /* UART 重启后必须收到新帧，不能沿用重启前的状态。 */
    now_ms = 1800u;
    send_payload(0x0201, status, sizeof(status));
    assert(referee_get_state(&state));
    referee_reset_stream();
    assert(!referee_get_state(&state) && !state.status.fresh);
    assert(state.generation != initial_generation);
    send_payload(0x0202, power, sizeof(power));
    assert(referee_get_state(&state) && !state.status.fresh && state.power_heat.fresh);
    uint32_t reset_generation = state.generation;
    status[0] = 3u;
    send_payload(0x0201, status, sizeof(status));
    assert(referee_get_state(&state) && state.status.fresh && !state.power_heat.fresh);
    assert(state.generation != reset_generation && state.robot_id == 3u);

    /* 32 位系统时间回绕，年龄仍应是 30ms。 */
    now_ms = UINT32_MAX - 20u;
    referee_init();
    send_payload(0x0201, status, sizeof(status));
    now_ms = 9u;
    assert(referee_get_state(&state) && state.status.age_ms == 30u);
}

static void test_gap_discards_partial_frame(void)
{
    now_ms = 0u;
    referee_init();
    uint8_t status[13] = {1u};
    uint8_t frame[32];
    size_t length = referee_protocol_encode(0x0201, 0u, status, sizeof(status), frame, sizeof(frame));
    referee_receive(frame, 10u, now_ms);
    now_ms = 200u;
    referee_receive(frame + 10u, length - 10u, now_ms);
    RefereeState state;
    assert(!referee_get_state(&state));
    referee_receive(frame, length, now_ms);
    assert(referee_get_state(&state) && state.robot_id == 1u);
}

int main(void)
{
    test_crc_vectors();
    test_stream();
    test_state();
    test_gap_discards_partial_frame();
    puts("referee: CRC vectors, framing/recovery, snapshots, timeouts PASS");
    return 0;
}
