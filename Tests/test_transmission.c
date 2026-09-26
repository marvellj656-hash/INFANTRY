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
    puts("transmission: BCP vectors, stream resync, timeout, generations and bounds passed");
    return 0;
}
