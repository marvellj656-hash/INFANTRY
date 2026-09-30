#include "referee_ui_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmd/cmd_mapping.h"
#include "referee/referee.h"
#include "referee/referee_task.h"
#include "referee_ui/referee_ui.h"

static uint32_t redraw_requests;
volatile RefereeUiDebug debug_referee_ui;
void referee_ui_request_redraw(void)
{
    taskENTER_CRITICAL();
    redraw_requests++;
    taskEXIT_CRITICAL();
}
void referee_ui_task_entry(void *argument)
{
    (void)argument;
    RefereeUiModel model;
    referee_ui_reset(&model);
    uint32_t handled_redraw = 0u;
    uint32_t previous_generation = 0u;
    uint16_t previous_client = 0u;
    bool was_online = false;
    TickType_t wake = xTaskGetTickCount();
    for (;;)
    {
        RefereeState referee;
        bool online = referee_get_state(&referee) && referee.status.fresh;
        uint16_t client = online ? referee_ui_client_id(referee.robot_id) : 0u;
        online = online && client != 0u;
        uint32_t request;
        taskENTER_CRITICAL();
        request = redraw_requests;
        taskEXIT_CRITICAL();
        /* 上线、换身份、R键三种情况都从ADD重新画起，不沿用旧客户端的“已画完”。 */
        if ((online && (!was_online || client != previous_client ||
                        referee.generation != previous_generation)) || request != handled_redraw)
        {
            referee_ui_reset(&model);
            handled_redraw = request;
            debug_referee_ui.redraws++;
        }
        if (online)
        {
            CmdOutput command = {0};
            bool fresh = cmd_get_state(&command);
            RefereeUiInput input = {
                .commands_fresh = fresh,
                .friction = command.shoot.enabled && command.shoot.friction,
                .spin = command.chassis.enabled && command.spin,
                .auto_aim = command.gimbal.enabled && command.gimbal.auto_aim,
                .heat_fresh = referee.power_heat.fresh && referee.status.fresh,
                .heat = referee.shooter_heat_17mm_1, .heat_limit = referee.shooter_heat_limit};
            RefereeUiPacket packet;
            if (referee_ui_prepare(&model, &input, &packet) &&
                referee_send_interaction(packet.data_command, client, packet.bytes, packet.length))
            {
                referee_ui_commit(&model, &packet);
                debug_referee_ui.queued++;
            }
            else
                debug_referee_ui.rejected++;
        }
        was_online = online;
        previous_client = client;
        previous_generation = referee.generation;
        debug_referee_ui.online = online;
        debug_referee_ui.client_id = client;
        debug_referee_ui.next_element = model.next_element;
        /* 每100ms最多1包；九个元素一圈900ms，让出时间给电机/姿态任务。 */
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(100u));
    }
}
