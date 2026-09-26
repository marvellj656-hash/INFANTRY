/*
 * 裁判协议迁移参考：旧 src/task/referee/referee_task.c
 * 原作者记录：ChenSihan (2024-01-19), SuperChen (2025-03-03)。
 * CRC 参数与旧 UI_Print/protocol/crc/crc.c 一致。
 * 此文件只有字节协议，不认识 UART、RTOS，也不控制电机。
 */
#include "referee_protocol.h"
#include <string.h>

static uint16_t read_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint8_t referee_crc8(const uint8_t *bytes, size_t length)
{
    uint8_t crc = 0xffu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint8_t)((crc >> 1) ^ ((crc & 1u) ? 0x8cu : 0u));
    }
    return crc;
}

uint16_t referee_crc16(const uint8_t *bytes, size_t length)
{
    uint16_t crc = 0xffffu;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc >> 1) ^ ((crc & 1u) ? 0x8408u : 0u));
    }
    return crc;
}

void referee_protocol_init(RefereeParser *parser)
{
    if (parser) memset(parser, 0, sizeof(*parser));
}

void referee_protocol_reset_stream(RefereeParser *parser)
{
    if (parser) parser->used = 0;
}

static void drop_prefix(RefereeParser *parser, uint16_t count)
{
    parser->used = (uint16_t)(parser->used - count);
    memmove(parser->bytes, parser->bytes + count, parser->used);
}

static void parse_available(RefereeParser *parser, RefereeFrameHandler handler, void *context)
{
    while (parser->used != 0u) {
        if (parser->bytes[0] != 0xa5u) {
            drop_prefix(parser, 1u);
            ++parser->discarded_bytes;
            continue;
        }
        if (parser->used < 5u) return;
        /* 先确认“信封标签”本身没有坏，再相信它写的包长。 */
        if (referee_crc8(parser->bytes, 4u) != parser->bytes[4]) {
            ++parser->header_errors;
            drop_prefix(parser, 1u);
            continue;
        }
        uint16_t payload_length = read_u16(parser->bytes + 1);
        if (payload_length > REFEREE_PAYLOAD_CAPACITY) {
            ++parser->length_errors;
            drop_prefix(parser, 1u);
            continue;
        }
        uint16_t frame_length = (uint16_t)(payload_length + 9u);
        if (parser->used < frame_length) return;
        if (referee_crc16(parser->bytes, frame_length - 2u) !=
            read_u16(parser->bytes + frame_length - 2u)) {
            ++parser->crc_errors;
            /* 只丢一个字节：坏包里面可能藏着下一包真正的 A5。 */
            drop_prefix(parser, 1u);
            continue;
        }
        RefereeFrame frame = {
            .command = read_u16(parser->bytes + 5),
            .length = payload_length,
            .sequence = parser->bytes[3],
            .payload = parser->bytes + 7
        };
        ++parser->valid_frames;
        if (handler) handler(&frame, context);
        drop_prefix(parser, frame_length);
    }
}

void referee_protocol_feed(RefereeParser *parser, const uint8_t *bytes,
                           size_t length, RefereeFrameHandler handler, void *context)
{
    if (!parser || (!bytes && length != 0u)) return;
    for (size_t i = 0; i < length; ++i) {
        /* parse_available 最迟在 128 字节时处理整包，所以不会越界。 */
        parser->bytes[parser->used++] = bytes[i];
        parse_available(parser, handler, context);
    }
}

size_t referee_protocol_encode(uint16_t command, uint8_t sequence,
                               const uint8_t *payload, size_t length,
                               uint8_t *out, size_t capacity)
{
    if (!out || (!payload && length != 0u) ||
        length > REFEREE_PAYLOAD_CAPACITY || capacity < length + 9u) return 0;
    out[0] = 0xa5u;
    out[1] = (uint8_t)length;
    out[2] = (uint8_t)(length >> 8);
    out[3] = sequence;
    out[4] = referee_crc8(out, 4u);
    out[5] = (uint8_t)command;
    out[6] = (uint8_t)(command >> 8);
    if (length) memcpy(out + 7, payload, length);
    uint16_t crc = referee_crc16(out, length + 7u);
    out[length + 7u] = (uint8_t)crc;
    out[length + 8u] = (uint8_t)(crc >> 8);
    return length + 9u;
}
