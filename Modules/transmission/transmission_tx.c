#include "transmission_tx.h"
#include <string.h>

void transmission_tx_init(TransmissionTx *tx)
{
    if (tx)
        memset(tx, 0, sizeof(*tx));
}

void transmission_tx_update(TransmissionTx *tx, uint32_t now_ms, bool ready,
                            const TransmissionMotion *motion,
                            TransmissionSend send, void *context)
{
    if (!tx)
        return;
    tx->ready = ready;
    tx->reference_valid = ready && motion != NULL;
    if (!motion)
        tx->motion_sent = false;
    if (!ready)
    {
        /* 再次连上时先报一次心跳，不必等云台归中。诊断计数保留。 */
        tx->motion_sent = tx->heartbeat_sent = false;
        return;
    }
    if (!send)
        return;
    if (TRANSMISSION_TX_HEARTBEAT_ENABLE && (!tx->heartbeat_sent ||
        now_ms - tx->last_heartbeat_ms >= TRANSMISSION_HEARTBEAT_PERIOD_MS))
    {
        uint8_t frame[TRANSMISSION_HEARTBEAT_FRAME_SIZE];
        size_t length = transmission_encode_heartbeat(frame, sizeof(frame), tx->next_heartbeat);
        if (send(frame, length, context))
        {
            tx->heartbeat = tx->next_heartbeat++;
            tx->heartbeat_frames++;
            tx->heartbeat_sent = true;
            tx->last_heartbeat_ms = now_ms;
        }
        else
            tx->heartbeat_busy++;
        /* 本圈只交一次，避免先发姿态把到期心跳一直挤掉。 */
        return;
    }
    if (!motion || (tx->motion_sent &&
        now_ms - tx->last_motion_ms < TRANSMISSION_ATTITUDE_PERIOD_MS))
        return;

    uint8_t frame[TRANSMISSION_RPY_FRAME_SIZE];
    size_t length = transmission_encode_motion(frame, sizeof(frame), motion->yaw_deg,
        motion->pitch_deg, motion->yaw_rate_deg_s, motion->pitch_rate_deg_s, motion->team);
    if (!length)
    {
        tx->invalid_samples++;
        tx->reference_valid = tx->motion_sent = false;
        return;
    }
    if (send(frame, length, context))
    {
        tx->last_motion = *motion;
        tx->last_motion_ms = now_ms;
        tx->motion_sent = true;
        tx->frames++;
    }
    else
        tx->busy++;
}
