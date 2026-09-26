#ifndef CMD_MAPPING_H
#define CMD_MAPPING_H
#include "chassis/chassis_task.h"
#include "gimbal/gimbal_control.h"
#include "remote/remote.h"
#include "shoot/shoot_control.h"
#include "transmission/transmission.h"

/* 拨杆线上传来的值不是从上到下 1、2、3，而是 1、3、2。 */
typedef enum {
  RC_SWITCH_UP = 1,
  RC_SWITCH_DOWN = 2,
  RC_SWITCH_MIDDLE = 3
} RcSwitch;
typedef struct {
  uint16_t previous_keys;
  uint8_t previous_left_switch;
  bool friction_inhibit, continuous_key, auto_active;
  uint32_t reference_generation, vision_generation;
  float auto_origin_yaw_deg, auto_origin_pitch_deg;
} CmdMapper;
typedef struct {
  RemoteState remote;
  GimbalOutput gimbal;
  TransmissionState vision;
  bool gimbal_fresh;
} CmdInput;
typedef struct {
  ChassisCommand chassis;
  GimbalCommand gimbal;
  ShootCommand shoot;
  bool spin;
  bool redraw_ui; /* R 键上升沿，只是重画请求，不改变电机许可。 */
} CmdOutput;
/* 纯映射：不读外设、不收队列。输入用快照，输出交给三个控制任务。 */
void cmd_map_update(CmdMapper *state, const CmdInput *input, CmdOutput *output);
/* 多读者快照；返回 false 表示遥控离线或指令发布已过期。 */
bool cmd_get_state(CmdOutput *out);
#endif
