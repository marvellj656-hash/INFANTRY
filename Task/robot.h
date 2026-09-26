#ifndef APP_ROBOT_H
#define APP_ROBOT_H

#include <stdbool.h>
/* MX_FREERTOS_Init 内调用，调度器启动前初始化模块并统一创建任务。 */
bool robot_init(void);

#endif
