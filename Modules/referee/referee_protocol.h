#ifndef MODULE_REFEREE_PROTOCOL_H
#define MODULE_REFEREE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 沿用旧 RTT 工程的 128 字节整包上限；不是“一个串口回调就是一包”。 */
#define REFEREE_FRAME_CAPACITY 128u
#define REFEREE_PAYLOAD_CAPACITY (REFEREE_FRAME_CAPACITY - 9u)

typedef struct {
    uint16_t command;
    uint16_t length;
    uint8_t sequence;
    const uint8_t *payload; /* 仅在回调里有效，需要保留时必须复制。 */
} RefereeFrame;

typedef void (*RefereeFrameHandler)(const RefereeFrame *frame, void *context);
typedef struct {
    uint8_t bytes[REFEREE_FRAME_CAPACITY];
    uint16_t used;
    uint32_t valid_frames;
    uint32_t header_errors;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t discarded_bytes;
} RefereeParser;

void referee_protocol_init(RefereeParser *parser);
/* UART 丢字节/重启时，只丢未完成的半包，保留累计诊断计数。 */
void referee_protocol_reset_stream(RefereeParser *parser);
void referee_protocol_feed(RefereeParser *parser, const uint8_t *bytes,
                           size_t length, RefereeFrameHandler handler, void *context);
uint8_t referee_crc8(const uint8_t *bytes, size_t length);
uint16_t referee_crc16(const uint8_t *bytes, size_t length);
/* 只管封包，调用者负责发送频率、发送对象与上层数据内容。失败返回 0。 */
size_t referee_protocol_encode(uint16_t command, uint8_t sequence,
                               const uint8_t *payload, size_t length,
                               uint8_t *out, size_t capacity);

#endif
