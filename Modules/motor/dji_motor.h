#ifndef MODULES_MOTOR_DJI_MOTOR_H
#define MODULES_MOTOR_DJI_MOTOR_H
#include "dji_protocol.h"
/* 机器人上的名字，不是 CAN ID，也不是电调拨码值。 */
typedef enum
{
    MOTOR_CHASSIS_FL,
    MOTOR_CHASSIS_FR,
    MOTOR_CHASSIS_RL,
    MOTOR_CHASSIS_RR,
    MOTOR_GIMBAL_YAW,
    MOTOR_GIMBAL_PITCH,
    MOTOR_TRIGGER,
    MOTOR_FRICTION_L,
    MOTOR_FRICTION_R,
    MOTOR_COUNT
} DjiMotorId;
typedef enum
{
    DJI_M3508,
    DJI_M2006,
    DJI_GM6020
} DjiMotorType;
typedef struct DjiMotor DjiMotor; /* 只给钥匙，不公开内部房间的布局。 */
typedef DjiMotor *DjiMotorHandle;
#define DJI_MOTOR_CAPACITY MOTOR_COUNT
typedef struct
{
    DjiMotorId id;
    DjiMotorType type;
    uint8_t bus;              /* 1=CAN1，2=CAN2。仅注册时需要接线知识。 */
    uint8_t esc_id;           /* 电调编号；模块推导反馈 ID、发送组和位置。 */
    int16_t output_limit_raw; /* 原始码值，不是安培；0=只读，禁止输出。 */
    uint32_t feedback_timeout_ms, command_timeout_ms;
} DjiMotorConfig;
typedef struct
{
    DjiFeedback measure;
    float total_angle_deg; /* 从本次连续反馈起点累计的转子角度，非输出轴角度。 */
    uint32_t generation;   /* 失联后重新计角时增加，位置控制器必须重置目标。 */
    uint32_t received_at_ms;
    bool received, online, transport_ready;
} DjiMotorFeedback;
typedef struct
{
    uint32_t bus, id;
    uint16_t ecd;
    int16_t speed_rpm, current_raw;
    uint8_t temperature;
    uint32_t online, age_ms, frames, invalid_frames;
} MotorFeedbackDebug;
extern volatile MotorFeedbackDebug debug_motor[MOTOR_COUNT];
/* 创建任务前注册，静态对象终生有效；重复名字/反馈ID/发送位置返回 NULL。
 * 不启动任务、不使能输出。失败不影响已注册对象。 */
DjiMotorHandle dji_motor_register(const DjiMotorConfig *config);
DjiMotorHandle dji_motor_find(DjiMotorId id);
/* 任务上下文调用。复制快照，多读者不抢消息；改副本不会改真实电机。
 * read_many 先复制，再取时间判在线。失败不改变 out；单个 read 失败清零 out。 */
bool dji_motor_read(DjiMotorHandle motor, DjiMotorFeedback *out);
bool dji_motor_read_many(const DjiMotorHandle *motors, size_t count, DjiMotorFeedback *out);
/* 一次提交一组控制结果，motor_task 统一组帧发送。
 * true=内存提交成功，不代表已经发送。每个电机仅允许一个控制任务写。
 * 零输出不受在线限制；只读对象拒绝非零。超时清除指令，需要重新提交。 */
bool dji_motor_set_outputs(const DjiMotorHandle *motors, const int16_t *raw, size_t count);
bool dji_motor_set_output(DjiMotorHandle motor, int16_t raw);
bool dji_motor_relax(DjiMotorHandle motor);
#endif
