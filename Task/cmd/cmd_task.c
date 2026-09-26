#include "cmd_task.h"
#include "FreeRTOS.h"
#include "bsp_time.h"
#include "cmd_mapping.h"
#include "gimbal/gimbal_task.h"
#include "referee_ui/referee_ui_task.h"
#include "robot_config.h"
#include "shoot/shoot_task.h"
#include "task.h"
volatile CmdDebug debug_cmd;
static CmdOutput published;
static bool published_online;
bool cmd_get_state(CmdOutput *out) {
  if (!out)
    return false;
  taskENTER_CRITICAL();
  *out = published;
  bool online = published_online;
  taskEXIT_CRITICAL();
  return online && (uint32_t)(bsp_time_ms() - out->chassis.received_at_ms) <
                       ROBOT_REMOTE_TIMEOUT_MS;
}
void cmd_task_entry(void *argument) {
  (void)argument;
  TickType_t wake = xTaskGetTickCount();
  CmdMapper mapper = {0};
  for (;;) {
    CmdInput input = {0};
    remote_update(ROBOT_REMOTE_TIMEOUT_MS, &input.remote);
    input.gimbal_fresh = gimbal_get_state(&input.gimbal);
    (void)transmission_get_state(&input.vision);
    CmdOutput output;
    cmd_map_update(&mapper, &input, &output);
    if (output.redraw_ui)
      referee_ui_request_redraw();
    taskENTER_CRITICAL();
    published = output;
    published_online = input.remote.online;
    taskEXIT_CRITICAL();
    /* 任务只接线：拿副本 -> 算命令 -> 提交。拨杆规则集中在 cmd_mapping.c。 */
    (void)chassis_set_command(&output.chassis);
    (void)gimbal_set_command(&output.gimbal);
    (void)shoot_set_command(&output.shoot);
    const DbusState *c = &input.remote.channels;
    debug_remote.enabled = output.chassis.enabled;
    debug_cmd = (CmdDebug){.online = input.remote.online,
                           .enabled = output.chassis.enabled,
                           .sw1 = c->sw1,
                           .sw2 = c->sw2,
                           .spin = output.spin,
                           .friction = output.shoot.friction,
                           .fire = output.shoot.fire,
                           .neutral = output.gimbal.neutral,
                           .auto_aim = output.gimbal.auto_aim,
                           .ch1 = c->ch[0],
                           .ch2 = c->ch[1],
                           .ch3 = c->ch[2],
                           .ch4 = c->ch[3],
                           .wheel = c->wheel};
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(ROBOT_CMD_PERIOD_MS));
  }
}
