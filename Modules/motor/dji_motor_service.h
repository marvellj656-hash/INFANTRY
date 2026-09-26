#ifndef DJI_MOTOR_SERVICE_H
#define DJI_MOTOR_SERVICE_H
#include "dji_motor.h"
/* 设备服务专用。控制任务只包含 dji_motor.h，不接触总线报文。 */
bool dji_motor_receive(uint8_t bus, uint16_t id, const uint8_t *data, size_t length,
                       uint32_t received_at_ms);
/* 唯一调用者 motor_task：检查超时，合并输出，发送每个活动组。 */
void dji_motor_service(void);
#endif
