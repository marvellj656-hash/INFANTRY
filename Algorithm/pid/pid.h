/**
 ******************************************************************************
 * @file     controller.h
 * @author  Wang Hongxi
 * @version V1.1.3
 * @date    2021/7/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef _PID_H
#define _PID_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* PID 对象由使用模块持有，不再维护 RTT 动态注册表。 */

#ifndef usr_abs
#define usr_abs(x) ((x > 0) ? x : -x)
#endif

#define INIT_PID_CONFIG(Kp_val, Ki_val, Kd_val, IntegralLimit_val, MaxOut_val, Improve_val) \
    {                                                                                \
        .Kp = Kp_val,                                                                \
        .Ki = Ki_val,                                                                \
        .Kd = Kd_val,                                                                \
        .IntegralLimit = IntegralLimit_val,                                          \
        .MaxOut = MaxOut_val,                                                        \
        .Improve = Improve_val,                                                      \
    }

/* PID 优化环节使能标志位,通过位与可以判断启用的优化环节 */
typedef enum
{
    PID_IMPROVE_NONE = 0X00,                // 0000 0000
    PID_Integral_Limit = 0x01,              // 0000 0001 积分限幅
    PID_Derivative_On_Measurement = 0x02,   // 0000 0010 微分先行
    PID_Trapezoid_Intergral = 0x04,         // 0000 0100 梯形积分
    PID_Proportional_On_Measurement = 0x08, // 0000 1000
    PID_OutputFilter = 0x10,                // 0001 0000 输出滤波
    PID_ChangingIntegrationRate = 0x20,     // 0010 0000 变速积分
    PID_DerivativeFilter = 0x40,            // 0100 0000 微分滤波器
    /* RTT控制语义：保留死区积分、原积分限幅及首次微分行为；仅云台/跟随使用。 */
    PID_RttControl = 0x100,
    PID_ErrorHandle = 0x80,                 // 1000 0000
} pid_improvement_e;

/* PID 报错类型枚举*/
typedef enum error_type_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} error_type_e;

typedef struct
{
    uint64_t error_count;
    error_type_e error_type;
} pid_ErrorHandler_t;

/* PID结构体 */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    // improve parameter
    pid_improvement_e Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分 For Changing Integral
    float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
    float Derivative_LPF_RC; // 微分滤波器系数

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    bool initialized; /* 首次计算不引入微分跳变。 */
    float dt;

    pid_ErrorHandler_t ERRORHandler;
} pid_obj_t;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区

    // improve parameter
    pid_improvement_e Improve;
    float IntegralLimit; // 积分限幅
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
} pid_config_t;

/**
 * @brief 初始化调用者提供的 PID 实例
 * @param pid 静态实例地址
 * @param config PID初始化配置
 */
/* 实例由调用者静态持有，不依赖 RT-Thread malloc 或注册表。 */
void pid_init(pid_obj_t *pid, const pid_config_t *config);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
/* dt 单位为秒，由任务传入；其余参数顺序沿用源码：反馈、目标。 */
float pid_calculate(pid_obj_t *pid, float measure, float ref, float dt);

/**
 * @brief 清空一个pid的历史数据
 *
 * @param pid    PID实例
 */
void pid_clear(pid_obj_t *pid);

#endif /* _PID_H */
