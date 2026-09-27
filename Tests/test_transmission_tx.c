#include "transmission/transmission_tx.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#undef assert
#define assert(condition) do { if (!(condition)) { fprintf(stderr, "check failed at line %d: %s\n", __LINE__, #condition); exit(1); } } while (0)

typedef struct
{
    bool busy;
    unsigned calls;
    size_t length;
    uint8_t frame[TRANSMISSION_MAX_FRAME_SIZE];
} FakeUsb;

static bool send_frame(const uint8_t *data, size_t length, void *context)
{
    FakeUsb *usb = context;
    assert(data && length <= sizeof(usb->frame));
    usb->calls++;
    if (usb->busy)
        return false;
    usb->length = length;
    memcpy(usb->frame, data, length);
    return true;
}

int main(void)
{
    TransmissionTx tx;
    FakeUsb usb = {0};
    TransmissionMotion motion = {.yaw_deg = -1.5f, .pitch_deg = 2.25f,
        .yaw_rate_deg_s = -123.5f, .pitch_rate_deg_s = 45.25f,
        .team = TRANSMISSION_TEAM_RED};
    transmission_tx_init(&tx);
    transmission_tx_update(&tx, 0u, false, NULL, send_frame, &usb);
    assert(usb.calls == 0u);

    /* USB已枚举但云台尚未归中：第一次就能报心跳，没有伪造姿态。 */
    usb.busy = true;
    transmission_tx_update(&tx, 1u, true, NULL, send_frame, &usb);
    assert(tx.heartbeat_busy == 1u && tx.heartbeat_frames == 0u && tx.frames == 0u);
    assert(tx.next_heartbeat == 0u);
    usb.busy = false;
    transmission_tx_update(&tx, 3u, true, NULL, send_frame, &usb);
    assert(tx.heartbeat_frames == 1u && tx.heartbeat == 0u && tx.next_heartbeat == 1u);
    assert(usb.length == 7u && usb.frame[2] == 0xf0u && usb.frame[4] == 0u);
    assert(transmission_checksum_valid(usb.frame, usb.length));
    transmission_tx_update(&tx, 5u, true, NULL, send_frame, &usb);
    assert(usb.calls == 2u && tx.frames == 0u);

    /* 姿态每10ms接受一次；busy不推进计时、不覆盖“实际发送”的调试样本。 */
    transmission_tx_update(&tx, 10u, true, &motion, send_frame, &usb);
    assert(tx.frames == 1u && tx.last_motion.yaw_deg == -1.5f && usb.frame[2] == 0x20u);
    unsigned before = usb.calls;
    transmission_tx_update(&tx, 19u, true, &motion, send_frame, &usb);
    assert(usb.calls == before);
    usb.busy = true;
    motion.yaw_deg = -2.0f;
    transmission_tx_update(&tx, 20u, true, &motion, send_frame, &usb);
    assert(tx.busy == 1u && tx.frames == 1u && tx.last_motion.yaw_deg == -1.5f);
    assert(tx.last_motion_ms == 10u);
    usb.busy = false;
    motion.yaw_deg = -3.0f;
    transmission_tx_update(&tx, 22u, true, &motion, send_frame, &usb);
    assert(tx.frames == 2u && tx.last_motion.yaw_deg == -3.0f && tx.last_motion_ms == 22u);
    assert(usb.frame[5] == 0x48 && usb.frame[6] == 0xf4); /* -3000 LE */

    /* 同时到期时，先交心跳，下圈交姿态，不在同一圈调用USB两次。 */
    before = usb.calls;
    transmission_tx_update(&tx, 103u, true, &motion, send_frame, &usb);
    assert(usb.calls == before + 1u && tx.heartbeat_frames == 2u && usb.frame[2] == 0xf0u);
    assert(tx.frames == 2u && tx.heartbeat == 1u);
    transmission_tx_update(&tx, 105u, true, &motion, send_frame, &usb);
    assert(tx.frames == 3u && usb.frame[2] == 0x20u);
    before = usb.calls;
    transmission_tx_update(&tx, 114u, true, &motion, send_frame, &usb);
    assert(usb.calls == before);
    transmission_tx_update(&tx, 115u, true, &motion, send_frame, &usb);
    assert(tx.frames == 4u);

    /* 非法姿态不交USB；不会借“最后发送值”伪装成新成功数据。 */
    motion.pitch_rate_deg_s = NAN;
    before = usb.calls;
    transmission_tx_update(&tx, 125u, true, &motion, send_frame, &usb);
    assert(usb.calls == before && tx.invalid_samples == 1u);
    assert(tx.last_motion.pitch_rate_deg_s == 45.25f && tx.frames == 4u);
    motion.pitch_rate_deg_s = 45.25f;

    /* 心跳也必须成功后才推进序号；再次尝试不会插入旧姿态。 */
    usb.busy = true;
    transmission_tx_update(&tx, 203u, true, &motion, send_frame, &usb);
    assert(tx.heartbeat_frames == 2u && tx.next_heartbeat == 2u && tx.heartbeat_busy == 2u);
    usb.busy = false;
    transmission_tx_update(&tx, 205u, true, &motion, send_frame, &usb);
    assert(tx.heartbeat_frames == 3u && tx.heartbeat == 2u && usb.frame[2] == 0xf0u);
    transmission_tx_update(&tx, 207u, true, &motion, send_frame, &usb);
    assert(tx.frames == 5u);

    /* 断线后的第一次ready立即报心跳。之前的计数仍可用来观察重连。 */
    transmission_tx_update(&tx, 208u, false, NULL, send_frame, &usb);
    transmission_tx_update(&tx, 209u, true, NULL, send_frame, &usb);
    assert(tx.heartbeat_frames == 4u && tx.heartbeat == 3u);

    /* 时间绕回和uint8序号绕回都合法；落后一秒也只发最新的一帧。 */
    transmission_tx_init(&tx);
    tx.next_heartbeat = 255u;
    transmission_tx_update(&tx, UINT32_MAX - 40u, true, NULL, send_frame, &usb);
    assert(tx.heartbeat == 255u && tx.next_heartbeat == 0u);
    before = usb.calls;
    transmission_tx_update(&tx, 58u, true, NULL, send_frame, &usb);
    assert(usb.calls == before);
    transmission_tx_update(&tx, 59u, true, NULL, send_frame, &usb);
    assert(tx.heartbeat == 0u && tx.heartbeat_frames == 2u);
    before = usb.calls;
    transmission_tx_update(&tx, 1059u, true, NULL, send_frame, &usb);
    assert(usb.calls == before + 1u && tx.heartbeat_frames == 3u);
    puts("transmission_tx: pre-home heartbeat, periods, priority, busy retry, fresh samples and wrap passed");
    return 0;
}
