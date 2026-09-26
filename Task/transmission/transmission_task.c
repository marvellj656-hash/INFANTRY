#include "transmission_task.h"
#include "transmission/transmission_service.h"
#include "referee/referee.h"
#include "ins/ins_task.h"
#include "gimbal/gimbal_task.h"
#include "bsp_nuc.h"
#include "bsp_time.h"
#include "FreeRTOS.h"
#include "task.h"

static TransmissionTeam team_from_referee(void)
{
    RefereeState state;
    if (!referee_get_state(&state) || !state.status.fresh)
        return TRANSMISSION_TEAM_UNKNOWN;
    if (state.robot_id >= 1u && state.robot_id <= 11u)
        return TRANSMISSION_TEAM_RED;
    if (state.robot_id >= 101u && state.robot_id <= 111u)
        return TRANSMISSION_TEAM_BLUE;
    return TRANSMISSION_TEAM_UNKNOWN;
}

void transmission_task_entry(void *argument)
{
    (void)argument;
    transmission_init();
    TickType_t wake = xTaskGetTickCount();
    uint32_t last_tx_ms = 0u, reference_generation = 0u, reference_changed_at_ms = 0u;
    float reference_yaw = 0.0f, reference_pitch = 0.0f, pc_pitch_origin = 0.0f;
    bool had_reference = false, received_reference_once = false;
    for (;;)
    {
        /* USB 的重活只在任务做。没有插 USB 时也不阻塞其他控制任务。 */
        bool started = bsp_nuc_start();
        uint32_t generation = bsp_nuc_generation();
        uint32_t now_ms = bsp_time_ms();
        transmission_service_update(now_ms, generation, started && bsp_nuc_ready());

        InsState ins;
        GimbalOutput gimbal;
        bool ins_ok = ins_get_state(&ins);
        bool gimbal_ok = gimbal_get_state(&gimbal);
        bool reference_ok = ins_ok && gimbal_ok && gimbal.centered &&
                            gimbal.reference_valid &&
                            gimbal.reference_generation == ins.generation;
        bool reference_changed = reference_ok &&
            (!had_reference || reference_generation != gimbal.reference_generation ||
             reference_yaw != gimbal.reference_yaw_deg ||
             reference_pitch != gimbal.reference_pitch_deg);
        if ((had_reference && !reference_ok) || reference_changed)
        {
            /* INS 重启/重新归中后，旧目标不能叠到新零点上。 */
            transmission_invalidate_target();
            reference_changed_at_ms = now_ms;
            received_reference_once = true;
        }
        if (reference_changed)
        {
            reference_generation = gimbal.reference_generation;
            reference_yaw = gimbal.reference_yaw_deg;
            reference_pitch = gimbal.reference_pitch_deg;
            pc_pitch_origin = reference_pitch;
        }
        had_reference = reference_ok;

        BspNucPacket packet;
        /* 每圈最多搬 16 个块，不让持续来的 USB 流量霸占 CPU。 */
        for (unsigned count = 0; count < 16u && bsp_nuc_receive(&packet); ++count)
        {
            uint32_t current_generation = bsp_nuc_generation();
            if (current_generation != generation)
            {
                transmission_service_update(bsp_time_ms(), current_generation, bsp_nuc_ready());
                generation = current_generation;
                continue;
            }
            if (received_reference_once &&
                (int32_t)(packet.received_at_ms - reference_changed_at_ms) <= 0)
                continue;
            transmission_service_receive(packet.data, packet.length,
                                         packet.received_at_ms, bsp_time_ms());
        }
        transmission_service_update(bsp_time_ms(), bsp_nuc_generation(), bsp_nuc_ready());

        if (reference_ok)
        {
            /* 保留旧行为：手动GYRO时pitch参考跟随当前姿态，AUTO时冻结。
             * yaw 则始终用归中时的参考；不要在驱动层把这两个约定混成同一种。 */
            if (gimbal.mode == GIMBAL_GYRO)
                pc_pitch_origin = ins.pitch_deg;
            if (now_ms - last_tx_ms >= 10u)
            {
                uint8_t frame[TRANSMISSION_RPY_FRAME_SIZE];
                size_t size = transmission_encode_attitude(
                    frame, sizeof(frame), reference_yaw - ins.yaw_deg,
                    ins.pitch_deg - pc_pitch_origin, team_from_referee());
                if (size && bsp_nuc_send(frame, size))
                    debug_transmission.tx_frames++;
                else
                    debug_transmission.tx_busy++;
                last_tx_ms = now_ms;
            }
        }
        if (!started)
        {
            vTaskDelay(pdMS_TO_TICKS(1000u));
            wake = xTaskGetTickCount();
        }
        else
            vTaskDelayUntil(&wake, pdMS_TO_TICKS(2u));
    }
}
