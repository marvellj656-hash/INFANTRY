#include "transmission_service.h"
#include "bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

static TransmissionParser parser;
static TransmissionTx transmitter;
static TransmissionState working, published;
static uint32_t last_frame_ms, last_chunk_ms, partial_started_ms, parsing_received_at_ms;
static bool frame_seen, target_seen, chunk_seen, transport_ready, heartbeat_seen;
volatile TransmissionDebug debug_transmission;

static void publish(uint32_t now_ms)
{
    working.age_ms = target_seen ? now_ms - working.received_at_ms : UINT32_MAX;
    working.heartbeat_age_ms = heartbeat_seen ? now_ms - working.heartbeat_at_ms : UINT32_MAX;
    working.heartbeat_online = transport_ready && heartbeat_seen &&
        working.heartbeat_age_ms < TRANSMISSION_HEARTBEAT_TIMEOUT_MS;
    working.online = transport_ready && frame_seen &&
                     now_ms - last_frame_ms < TRANSMISSION_LINK_TIMEOUT_MS;
    working.aim_valid = working.online && target_seen &&
                        working.age_ms < TRANSMISSION_TARGET_TIMEOUT_MS;
    if (!working.aim_valid)
        working.fire_advice = false;
    taskENTER_CRITICAL();
    published = working;
    taskEXIT_CRITICAL();
    debug_transmission.age_ms = working.age_ms;
    debug_transmission.command_generation = working.command_generation;
    debug_transmission.yaw_deg = working.yaw_deg;
    debug_transmission.pitch_deg = working.pitch_deg;
    debug_transmission.raw_target_flag = working.raw_target_flag;
    debug_transmission.heartbeat = working.heartbeat;
    debug_transmission.heartbeat_at_ms = working.heartbeat_at_ms;
    debug_transmission.heartbeat_age_ms = working.heartbeat_age_ms;
    debug_transmission.heartbeat_online = working.heartbeat_online;
    debug_transmission.destination = working.destination;
    debug_transmission.online = working.online;
    debug_transmission.aim_valid = working.aim_valid;
    debug_transmission.fire_advice = working.fire_advice;
    debug_transmission.checksum_errors = parser.checksum_errors;
    debug_transmission.length_errors = parser.length_errors;
    debug_transmission.discarded_bytes = parser.discarded_bytes;
}

void transmission_init(void)
{
    transmission_parser_reset(&parser);
    transmission_tx_init(&transmitter);
    memset(&working, 0, sizeof(working));
    memset(&published, 0, sizeof(published));
    memset((void *)&debug_transmission, 0, sizeof(debug_transmission));
    working.age_ms = published.age_ms = UINT32_MAX;
    working.heartbeat_age_ms = published.heartbeat_age_ms = UINT32_MAX;
    debug_transmission.heartbeat_age_ms = UINT32_MAX;
    debug_transmission.tx_attitude_age_ms = UINT32_MAX;
    last_frame_ms = last_chunk_ms = partial_started_ms = parsing_received_at_ms = 0u;
    frame_seen = target_seen = chunk_seen = transport_ready = heartbeat_seen = false;
}

void transmission_invalidate_target(void)
{
    target_seen = false;
    working.aim_valid = working.fire_advice = false;
    working.yaw_deg = working.pitch_deg = 0.0f;
    parser.used = 0u;
    chunk_seen = false;
    publish(bsp_time_ms());
}

void transmission_service_update(uint32_t now_ms, uint32_t generation, bool ready)
{
    if (generation != working.link_generation || !ready)
    {
        /* 即使USB很快重新ready、任务没看到断开瞬间，也不能沿用旧发送有效标记。 */
        transmission_service_send(now_ms, false, NULL, NULL, NULL);
        if (generation != working.link_generation)
            debug_transmission.generation_resets++;
        working.link_generation = generation;
        target_seen = frame_seen = chunk_seen = heartbeat_seen = false;
        working.heartbeat = 0u;
        working.heartbeat_at_ms = 0u;
        working.heartbeat_online = false;
        working.aim_valid = working.fire_advice = false;
        working.yaw_deg = working.pitch_deg = 0.0f;
        parser.used = 0u;
    }
    transport_ready = ready;
    if (target_seen && now_ms - working.received_at_ms >= TRANSMISSION_TARGET_TIMEOUT_MS)
    {
        /* 超时要撤销“见过目标”。之后只有心跳恢复，不能把旧角度重新点亮。 */
        target_seen = false;
        working.aim_valid = working.fire_advice = false;
        debug_transmission.target_timeouts++;
    }
    if (chunk_seen && (now_ms - last_chunk_ms >= TRANSMISSION_TARGET_TIMEOUT_MS ||
        (parser.used && now_ms - partial_started_ms >= TRANSMISSION_TARGET_TIMEOUT_MS)))
    {
        parser.used = 0u;
        chunk_seen = false;
    }
    publish(now_ms);
}

static void accept_frame(const TransmissionFrame *frame, void *context)
{
    (void)context;
    /* 旧上位机没有提供发包源码，旧 MCU 也没有检查地址。
     * 先保持兼容，只记录地址；抓到真实报文后再约定本机地址过滤。 */
    working.destination = frame->destination;
    if (frame->id == TRANSMISSION_GIMBAL_ID)
    {
        TransmissionTarget target;
        if (!transmission_decode_target(frame, &target))
        {
            /* 一个结构完整但内容非法的目标包明确中止之前的目标。 */
            target_seen = false;
            working.aim_valid = working.fire_advice = false;
            debug_transmission.invalid_frames++;
            return;
        }
        working.yaw_deg = target.yaw_deg;
        working.pitch_deg = target.pitch_deg;
        working.fire_advice = target.fire_advice;
        working.raw_target_flag = target.raw_target_flag;
        working.received_at_ms = parsing_received_at_ms;
        working.command_generation++;
        target_seen = true;
        debug_transmission.target_frames++;
    }
    else if (frame->id == TRANSMISSION_HEARTBEAT_ID &&
             (frame->length == 1u || frame->length == TRANSMISSION_RPY_DATA_SIZE))
    {
        /* 旧实现按固定 31 字节收心跳；同时支持 LEN=1 的正常短心跳。 */
        working.heartbeat = frame->data[0];
        working.heartbeat_at_ms = parsing_received_at_ms;
        working.heartbeat_frames++;
        heartbeat_seen = true;
        debug_transmission.heartbeat_frames++;
    }
    else
    {
        debug_transmission.invalid_frames++;
        return;
    }
    frame_seen = true;
    last_frame_ms = parsing_received_at_ms;
    working.last_frame_at_ms = last_frame_ms;
    debug_transmission.valid_frames++;
}

void transmission_service_receive(const uint8_t *data, size_t length,
                                  uint32_t received_at_ms, uint32_t now_ms)
{
    if (!transport_ready || !data || !length)
        return;
    if (now_ms - received_at_ms >= TRANSMISSION_TARGET_TIMEOUT_MS)
    {
        /* 队列积压的旧字节不许被当作刚收到的新目标。 */
        parser.used = 0u;
        chunk_seen = false;
        return;
    }
    if (chunk_seen && (received_at_ms - last_chunk_ms >= TRANSMISSION_TARGET_TIMEOUT_MS ||
        (parser.used && received_at_ms - partial_started_ms >= TRANSMISSION_TARGET_TIMEOUT_MS)))
        parser.used = 0u;
    if (!parser.used)
        partial_started_ms = received_at_ms;
    uint32_t frames_before = parser.frames;
    parsing_received_at_ms = last_chunk_ms = received_at_ms;
    chunk_seen = true;
    debug_transmission.rx_bytes += (uint32_t)length;
    transmission_parser_feed(&parser, data, length, accept_frame, NULL);
    /* 一个块也可能是“上一帧尾巴+下一帧头”，新半帧从本块开始计时。 */
    if (parser.frames != frames_before)
        partial_started_ms = received_at_ms;
    publish(now_ms);
}

bool transmission_get_state(TransmissionState *out)
{
    if (!out)
        return false;
    taskENTER_CRITICAL();
    *out = published;
    taskEXIT_CRITICAL();
    uint32_t now_ms = bsp_time_ms();
    /* 任务被饿住也不能让读者永远看到 valid=true；读副本时再次验时间。 */
    if (out->age_ms != UINT32_MAX)
        out->age_ms = now_ms - out->received_at_ms;
    if (out->heartbeat_age_ms != UINT32_MAX)
        out->heartbeat_age_ms = now_ms - out->heartbeat_at_ms;
    out->heartbeat_online = out->heartbeat_online &&
        out->heartbeat_age_ms < TRANSMISSION_HEARTBEAT_TIMEOUT_MS;
    out->aim_valid = out->aim_valid && out->age_ms < TRANSMISSION_TARGET_TIMEOUT_MS;
    out->online = out->online && now_ms - out->last_frame_at_ms < TRANSMISSION_LINK_TIMEOUT_MS;
    out->fire_advice = out->fire_advice && out->aim_valid;
    return out->aim_valid;
}

void transmission_service_send(uint32_t now_ms, bool ready, const TransmissionMotion *motion,
                               TransmissionSend send, void *context)
{
    transmission_tx_update(&transmitter, now_ms, ready, motion, send, context);
    debug_transmission.tx_frames = transmitter.frames;
    debug_transmission.tx_busy = transmitter.busy;
    debug_transmission.tx_invalid_samples = transmitter.invalid_samples;
    debug_transmission.tx_heartbeat_frames = transmitter.heartbeat_frames;
    debug_transmission.tx_heartbeat_busy = transmitter.heartbeat_busy;
    debug_transmission.tx_heartbeat = transmitter.heartbeat;
    debug_transmission.tx_ready = transmitter.ready;
    debug_transmission.tx_reference_valid = transmitter.reference_valid;
    /* 角度字段保留最后一次真实接受的样本，用age/valid说明还能否用它。 */
    debug_transmission.tx_attitude_age_ms = transmitter.reference_valid && transmitter.motion_sent ?
        now_ms - transmitter.last_motion_ms : UINT32_MAX;
    debug_transmission.tx_yaw_deg = transmitter.last_motion.yaw_deg;
    debug_transmission.tx_pitch_deg = transmitter.last_motion.pitch_deg;
    debug_transmission.tx_yaw_rate_deg_s = transmitter.last_motion.yaw_rate_deg_s;
    debug_transmission.tx_pitch_rate_deg_s = transmitter.last_motion.pitch_rate_deg_s;
}
