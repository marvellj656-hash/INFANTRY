#include "robot.h"
#include "FreeRTOS.h"
#include "bsp_can.h"
#include "chassis/chassis_config.h"
#include "chassis/chassis_task.h"
#include "cmd/cmd_task.h"
#include "gimbal/gimbal_task.h"
#include "ins/ins_task.h"
#include "motor/dji_motor.h"
#include "motor/motor_task.h"
#include "referee/referee_task.h"
#include "referee_ui/referee_ui_task.h"
#include "remote/remote.h"
#include "robot_config.h"
#include "shoot/shoot_task.h"
#include "shoot/shoot_config.h"
#include "task.h"
/* CubeMX 旧窗口曾把堆恢复为15360：编译时就提醒，避免上电才卡在任务创建失败。 */
_Static_assert(configTOTAL_HEAP_SIZE >= 32768u,
               "Set FreeRTOS TOTAL_HEAP_SIZE >= 32768 in CubeMX before generating.");
#include "transmission/transmission_task.h"
/* 9个电机
 * 接线只在这里声明。编号是电调编号，发送组由电机模块自动分配。
 * 任务只通过语义 ID 使用设备。 */
static const DjiMotorConfig motor_config[] = {
    {.id = MOTOR_CHASSIS_FL,
     .type = DJI_M3508,
     .bus = 1,
     .esc_id = 1,
     .output_limit_raw = CHASSIS_CURRENT_LIMIT_RAW,
     .feedback_timeout_ms = ROBOT_MOTOR_FEEDBACK_TIMEOUT_MS,
     .command_timeout_ms = ROBOT_MOTOR_COMMAND_TIMEOUT_MS},
    {.id = MOTOR_CHASSIS_FR,
     .type = DJI_M3508,
     .bus = 1,
     .esc_id = 2,
     .output_limit_raw = CHASSIS_CURRENT_LIMIT_RAW,
     .feedback_timeout_ms = ROBOT_MOTOR_FEEDBACK_TIMEOUT_MS,
     .command_timeout_ms = ROBOT_MOTOR_COMMAND_TIMEOUT_MS},
    {.id = MOTOR_CHASSIS_RL,
     .type = DJI_M3508,
     .bus = 1,
     .esc_id = 3,
     .output_limit_raw = CHASSIS_CURRENT_LIMIT_RAW,
     .feedback_timeout_ms = ROBOT_MOTOR_FEEDBACK_TIMEOUT_MS,
     .command_timeout_ms = ROBOT_MOTOR_COMMAND_TIMEOUT_MS},
    {.id = MOTOR_CHASSIS_RR,
     .type = DJI_M3508,
     .bus = 1,
     .esc_id = 4,
     .output_limit_raw = CHASSIS_CURRENT_LIMIT_RAW,
     .feedback_timeout_ms = ROBOT_MOTOR_FEEDBACK_TIMEOUT_MS,
     .command_timeout_ms = ROBOT_MOTOR_COMMAND_TIMEOUT_MS},

    {.id = MOTOR_GIMBAL_YAW,
     .type = DJI_GM6020,
     .bus = 1,
     .esc_id = 3,
     .output_limit_raw = 30000,
     .feedback_timeout_ms = ROBOT_MOTOR_FEEDBACK_TIMEOUT_MS,
     .command_timeout_ms = ROBOT_MOTOR_COMMAND_TIMEOUT_MS},
    {.id = MOTOR_GIMBAL_PITCH,
     .type = DJI_GM6020,
     .bus = 2,
     .esc_id = 4,
     .output_limit_raw = 30000,
     .feedback_timeout_ms = ROBOT_MOTOR_FEEDBACK_TIMEOUT_MS,
     .command_timeout_ms = ROBOT_MOTOR_COMMAND_TIMEOUT_MS},

    {.id = MOTOR_FRICTION_L,
     .type = DJI_M3508,
     .bus = 2,
     .esc_id = FRICTION_LEFT_ESC_ID,
     .output_limit_raw = 16000,
     .feedback_timeout_ms = ROBOT_MOTOR_FEEDBACK_TIMEOUT_MS,
     .command_timeout_ms = ROBOT_MOTOR_COMMAND_TIMEOUT_MS},
    {.id = MOTOR_FRICTION_R,
     .type = DJI_M3508,
     .bus = 2,
     .esc_id = FRICTION_RIGHT_ESC_ID,
     .output_limit_raw = 16000,
     .feedback_timeout_ms = ROBOT_MOTOR_FEEDBACK_TIMEOUT_MS,
     .command_timeout_ms = ROBOT_MOTOR_COMMAND_TIMEOUT_MS},

    {.id = MOTOR_TRIGGER,
     .type = DJI_M2006,
     .bus = 2,
     .esc_id = 3,
     .output_limit_raw = 10000,
     .feedback_timeout_ms = ROBOT_MOTOR_FEEDBACK_TIMEOUT_MS,
     .command_timeout_ms = ROBOT_MOTOR_COMMAND_TIMEOUT_MS}};
bool robot_init(void) {
  /* 所有资源先准备好，任务才开始跑；失败交给已有启动错误处理。 */
  if (!remote_init() || !bsp_can_init())
    return false;
  for (unsigned i = 0; i < sizeof(motor_config) / sizeof(motor_config[0]); ++i)
    if (!dji_motor_register(&motor_config[i]))
      return false;
  if (!chassis_init() || !gimbal_init() || !shoot_init() ||
      !referee_task_init())
    return false;
  if (xTaskCreate(motor_task_entry, "motor_io", 512, NULL, 3, NULL) != pdPASS)
    return false;
  if (xTaskCreate(cmd_task_entry, "cmd", 512, NULL, 3, NULL) != pdPASS)
    return false;
  if (xTaskCreate(ins_task_entry, "ins", 1024, NULL, 3, NULL) != pdPASS)
    return false;
  if (xTaskCreate(gimbal_task_entry, "gimbal", 768, NULL, 4, NULL) != pdPASS)
    return false;
  if (xTaskCreate(shoot_task_entry, "shoot", 512, NULL, 4, NULL) != pdPASS)
    return false;
  /* 通信任务优先级低于控制环，USB拔插/裁判拆包不能抢占2ms电机控制。 */
  if (xTaskCreate(referee_task_entry, "referee", 512, NULL, 2, NULL) != pdPASS)
    return false;
  if (xTaskCreate(transmission_task_entry, "nuc", 768, NULL, 2, NULL) != pdPASS)
    return false;
  if (xTaskCreate(referee_ui_task_entry, "referee_ui", 512, NULL, 1, NULL) !=
      pdPASS)
    return false;
  return xTaskCreate(chassis_task_entry, "chassis", 512, NULL, 4, NULL) ==
         pdPASS;
}
