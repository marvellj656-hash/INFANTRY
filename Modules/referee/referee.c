/* 旧裁判模块原作者：ChenSihan / SuperChen，参见 referee_protocol.c。 */
#include "referee.h"
#include "referee_protocol.h"
#include "bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

static RefereeParser parser;
static RefereeState state;
static RefereeState published;
static uint32_t packet_time;
static uint32_t previous_packet_time;
static bool seen_packet;
volatile RefereeDebug debug_referee;

static uint16_t u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static uint32_t u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static float f32(const uint8_t *p)
{
    uint32_t bits = u32(p);
    float value;
    _Static_assert(sizeof(float) == sizeof(uint32_t), "referee requires 32-bit float");
    memcpy(&value, &bits, sizeof(value));
    return value;
}
static void stamp(RefereeFreshness *freshness)
{
    freshness->received = true;
    freshness->received_at_ms = packet_time;
}
static bool expected_length(const RefereeFrame *frame, uint16_t length)
{
    if (frame->length == length) return true;
    ++debug_referee.invalid_payloads;
    return false;
}

static void handle_frame(const RefereeFrame *frame, void *context)
{
    (void)context;
    const uint8_t *p = frame->payload;
    state.online = true;
    state.received_at_ms = packet_time;
    ++state.valid_frames;
    state.last_command = frame->command;
    state.last_sequence = frame->sequence;
    /* 只按旧 rm_task.h 的实际成员宽度读取。
     * 收到别的长度，宁可报错，也不能把新布局错读成旧功率值。 */
    switch (frame->command) {
    case 0x0001:
        if (!expected_length(frame, 11u)) break;
        state.game_type = p[0] & 0x0fu;
        state.game_progress = p[0] >> 4;
        state.stage_remaining_s = u16(p + 1);
        state.sync_timestamp = (uint64_t)u32(p + 3) | ((uint64_t)u32(p + 7) << 32);
        stamp(&state.game);
        break;
    case 0x0003:
        if (!expected_length(frame, 32u)) break;
        for (unsigned i = 0; i < 16; ++i) state.team_robot_hp[i] = u16(p + 2u * i);
        stamp(&state.robot_hp);
        break;
    case 0x0101:
        if (!expected_length(frame, 4u)) break;
        state.event_bits = u32(p);
        stamp(&state.event);
        break;
    case 0x0201:
        if (!expected_length(frame, 13u)) break;
        if (state.robot_id != p[0]) {
            ++state.generation;
            /* 换到另一台机器人后，不能把上一台的枪口热量和余弹继承过去。 */
            state.power_heat.received = false;
            state.shooting.received = false;
            state.allowance.received = false;
        }
        state.robot_id = p[0];
        state.robot_level = p[1];
        state.current_hp = u16(p + 2);
        state.maximum_hp = u16(p + 4);
        state.shooter_cooling_per_s = u16(p + 6);
        state.shooter_heat_limit = u16(p + 8);
        state.chassis_power_limit_w = u16(p + 10);
        state.gimbal_power_enabled = (p[12] & 1u) != 0;
        state.chassis_power_enabled = (p[12] & 2u) != 0;
        state.shooter_power_enabled = (p[12] & 4u) != 0;
        stamp(&state.status);
        break;
    case 0x0202:
        if (!expected_length(frame, 16u)) break;
        state.legacy_voltage_raw = u16(p);
        state.legacy_current_raw = u16(p + 2);
        state.legacy_power_bits = u32(p + 4);
        state.buffer_energy_j = u16(p + 8);
        state.shooter_heat_17mm_1 = u16(p + 10);
        state.shooter_heat_17mm_2 = u16(p + 12);
        state.shooter_heat_42mm = u16(p + 14);
        stamp(&state.power_heat);
        break;
    case 0x0207:
        if (!expected_length(frame, 7u)) break;
        if (!isfinite(f32(p + 3)) || f32(p + 3) < 0.0f) {
            ++debug_referee.invalid_payloads;
            break;
        }
        state.bullet_type = p[0];
        state.shooter_number = p[1];
        state.launching_frequency_hz = p[2];
        state.bullet_speed_m_s = f32(p + 3);
        stamp(&state.shooting);
        break;
    case 0x0208:
        if (!expected_length(frame, 6u)) break;
        state.projectile_allowance_17mm = u16(p);
        state.projectile_allowance_42mm = u16(p + 2);
        state.remaining_gold_coin = u16(p + 4);
        stamp(&state.allowance);
        break;
    default:
        /* 收到不使用的命令仍算 UART 链路存活，但不能刷新其他数据年龄。 */
        ++debug_referee.unsupported_commands;
        break;
    }
}

static void calculate_freshness(RefereeFreshness *f, uint32_t now, uint32_t timeout)
{
    f->age_ms = f->received ? now - f->received_at_ms : UINT32_MAX;
    f->fresh = f->received && f->age_ms < timeout;
}
static void refresh(RefereeState *out, uint32_t now)
{
    out->age_ms = out->valid_frames ? now - out->received_at_ms : UINT32_MAX;
    out->online = out->valid_frames != 0u && out->age_ms < REFEREE_LINK_TIMEOUT_MS;
    calculate_freshness(&out->status, now, REFEREE_STATUS_TIMEOUT_MS);
    calculate_freshness(&out->power_heat, now, REFEREE_POWER_TIMEOUT_MS);
    calculate_freshness(&out->shooting, now, REFEREE_LINK_TIMEOUT_MS);
    calculate_freshness(&out->allowance, now, REFEREE_STATUS_TIMEOUT_MS);
    calculate_freshness(&out->game, now, REFEREE_STATUS_TIMEOUT_MS);
    calculate_freshness(&out->robot_hp, now, REFEREE_STATUS_TIMEOUT_MS);
    calculate_freshness(&out->event, now, REFEREE_STATUS_TIMEOUT_MS);
}

void referee_init(void)
{
    referee_protocol_init(&parser);
    memset(&state, 0, sizeof(state));
    memset((void *)&debug_referee, 0, sizeof(debug_referee));
    seen_packet = false;
    previous_packet_time = 0;
    packet_time = 0;
    referee_update();
}

void referee_reset_stream(void)
{
    referee_protocol_reset_stream(&parser);
    seen_packet = false;
    ++debug_referee.stream_resets;
    ++state.generation;
    /* 历史值可以留下看，但 UART 重启后不能继续标成 fresh。 */
    state.valid_frames = 0;
    state.status.received = false;
    state.power_heat.received = false;
    state.shooting.received = false;
    state.allowance.received = false;
    state.game.received = false;
    state.robot_hp.received = false;
    state.event.received = false;
    referee_update();
}

void referee_receive(const uint8_t *bytes, size_t length, uint32_t received_at_ms)
{
    if (!bytes || length == 0u) return;
    if (seen_packet && (uint32_t)(received_at_ms - previous_packet_time) > REFEREE_STREAM_GAP_MS)
        referee_protocol_reset_stream(&parser);
    seen_packet = true;
    previous_packet_time = received_at_ms;
    packet_time = received_at_ms;
    debug_referee.rx_bytes += (uint32_t)length;
    referee_protocol_feed(&parser, bytes, length, handle_frame, NULL);
    referee_update();
}

void referee_update(void)
{
    refresh(&state, bsp_time_ms());
    /* 临界区只复制约 200 字节。CRC 循环和 UART 操作都在外面执行。 */
    taskENTER_CRITICAL();
    published = state;
    taskEXIT_CRITICAL();
    debug_referee.valid_frames = parser.valid_frames;
    debug_referee.header_errors = parser.header_errors;
    debug_referee.crc_errors = parser.crc_errors;
    debug_referee.length_errors = parser.length_errors;
    debug_referee.age_ms = state.age_ms;
    debug_referee.last_command = state.last_command;
    debug_referee.chassis_power_limit_w = state.chassis_power_limit_w;
    debug_referee.buffer_energy_j = state.buffer_energy_j;
    debug_referee.heat_17mm_1 = state.shooter_heat_17mm_1;
    debug_referee.heat_limit = state.shooter_heat_limit;
    debug_referee.bullet_speed_m_s = state.bullet_speed_m_s;
    debug_referee.robot_id = state.robot_id;
    debug_referee.online = state.online;
    debug_referee.status_fresh = state.status.fresh;
    debug_referee.power_heat_fresh = state.power_heat.fresh;
}

bool referee_get_state(RefereeState *out)
{
    if (!out) return false;
    taskENTER_CRITICAL();
    *out = published;
    taskEXIT_CRITICAL();
    /* 读取者自己算一次年龄：即便接收任务停住，旧值也会按时过期。 */
    refresh(out, bsp_time_ms());
    return out->online;
}
