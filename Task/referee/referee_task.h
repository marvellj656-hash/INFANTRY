#ifndef APP_REFEREE_TASK_H
#define APP_REFEREE_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* 在创建任务前调用一次；初始化模块和静态发送队列。 */
bool referee_task_init(void);
void referee_task_entry(void *argument);
/* 上层 UI/机器人交互把数据交给任务，自己不用碰 USART6。
 * data_command 是 0x0301 内层命令；模块自动填写真实 sender robot_id。
 * 返回 true 只表示排进队列，成功交给 UART 的数量看 debug_referee.tx_accepted。
 * UART 暂忙保留请求重试；离线或身份/代次变化会清理旧请求。
 * 暂不包含 UI 绘图对象、图层刷新策略和具体赛季发送配额。 */
bool referee_send_interaction(uint16_t data_command, uint16_t receiver_id,
                              const uint8_t *data, size_t length);

#endif
