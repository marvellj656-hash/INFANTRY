#ifndef BSP_MOTOR_CAN_H
#define BSP_MOTOR_CAN_H
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
typedef struct {
    TickType_t received_at;//time
    uint16_t id;//id
    uint8_t bus;//BUS1.2
    uint8_t length;//dcl
    uint8_t data[8];
} MotorCanPacket;
typedef struct {
    uint32_t started[2];
    uint32_t rx_frames[2];
    uint32_t queue_drops[2];
    uint32_t rejected[2];
    uint32_t errors[2];
    uint32_t last_error[2];
    uint32_t start_failures[2];
} MotorCanDebug;
extern volatile MotorCanDebug debug_motor_can;
bool bsp_motor_can_init(void);
/* 任务上下文：为 CubeMX 已初始化的 CAN1 配置过滤器并启动接收。 */
bool bsp_motor_can_start(void);
bool bsp_motor_can_take(MotorCanPacket *packet);
typedef struct {
    uint32_t queued, cancel_requests, busy, errors;
} ChassisCanTxDebug;
extern volatile ChassisCanTxDebug debug_chassis_tx;
bool bsp_motor_can_ready(void);
bool bsp_motor_can_send_chassis(const int16_t current_raw[4]);
#endif
