/**
 * @file controller.c
 * @author wanghongxi
 * @author modified by neozng
 * @brief  PID控制器定义
 * @version beta
 * @date 2022-11-01
 *
 * @copyrightCopyright (c) 2022 HNU YueLu EC all rights reserved
 */

#include "pid.h"
#include <string.h>

/* ----------------------------以下是pid优化环节的实现---------------------------- */

// 梯形积分
static void f_Trapezoid_Intergral(pid_obj_t *pid)
{
    // 计算梯形的面积,(上底+下底)*高/2
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2) *  pid->dt;
}

// 变速积分(误差小时积分作用更强)
static void f_Changing_Integration_Rate(pid_obj_t *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        // 积分呈累积趋势
        if (usr_abs(pid->Err) <= pid->CoefB)
            return; // Full integral
        if (pid->CoefA > 0 && usr_abs(pid->Err) <= (pid->CoefA + pid->CoefB))
            pid->ITerm *= (pid->CoefA - usr_abs(pid->Err) + pid->CoefB) / pid->CoefA;
        else // 最大阈值,不使用积分
            pid->ITerm = 0;
    }
}

static void f_Integral_Limit(pid_obj_t *pid)
{
    float temp_Output, temp_Iout; /* 各实例不共享临时状态。 */
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + temp_Iout + pid->Dout;
    if (usr_abs(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * temp_Output > 0) // 预计饱和，停止同方向积分
        {
            pid->ITerm = 0; // 当前积分项置零
            temp_Iout = pid->Iout;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

// 微分先行(仅使用反馈值而不计参考输入的微分)
static void f_Derivative_On_Measurement(pid_obj_t *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure) / pid->dt;
}

// 微分滤波(采集微分时,滤除高频噪声)
static void f_Derivative_Filter(pid_obj_t *pid)
{
    pid->Dout = pid->Dout * pid->dt / (pid->Derivative_LPF_RC + pid->dt) +
                pid->Last_Dout * pid->Derivative_LPF_RC / (pid->Derivative_LPF_RC + pid->dt);
}

// 输出滤波
static void f_Output_Filter(pid_obj_t *pid)
{
    pid->Output = pid->Output * pid->dt / (pid->Output_LPF_RC + pid->dt) +
                  pid->Last_Output * pid->Output_LPF_RC / (pid->Output_LPF_RC + pid->dt);
}

// 输出限幅
static void f_Output_Limit(pid_obj_t *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

// 电机堵转检测
static void f_PID_ErrorHandle(pid_obj_t *pid)
{
    /*Motor Blocked Handle*/
    if (fabsf(pid->Output) < pid->MaxOut * 0.001f || fabsf(pid->Ref) < 0.0001f)
    {
        pid->ERRORHandler.error_count = 0;
        return;
    }

    if ((fabsf(pid->Ref - pid->Measure) / fabsf(pid->Ref)) > 0.95f)
    {
        // Motor blocked counting
        pid->ERRORHandler.error_count++;
    }
    else
    {
        pid->ERRORHandler.error_count = 0;
    }

    if (pid->ERRORHandler.error_count > 500)
    {
        // 连续超过500次满足判据；这里只记录错误，不自动控制电机。
        pid->ERRORHandler.error_type = PID_MOTOR_BLOCKED_ERROR;
    }
}

/* ---------------------------下面是PID的外部算法接口--------------------------- */

/**
 * @brief 初始化调用者提供的 PID 实例
 * @param config PID初始化设置
 */
void pid_init(pid_obj_t *object, const pid_config_t *config)
{
    memset(object, 0, sizeof(*object));
    object->Kp=config->Kp; object->Ki=config->Ki; object->Kd=config->Kd;
    object->MaxOut=config->MaxOut; object->DeadBand=config->DeadBand;
    object->Improve=config->Improve; object->IntegralLimit=config->IntegralLimit;
    object->CoefA=config->CoefA; object->CoefB=config->CoefB;
    object->Output_LPF_RC=config->Output_LPF_RC;
    object->Derivative_LPF_RC=config->Derivative_LPF_RC;
}

/**
 * @brief          PID计算
 * @param[in]      PID结构体
 * @param[in]      测量值
 * @param[in]      期望值
 * @retval         返回空
 */
float pid_calculate(pid_obj_t *pid, float measure, float ref, float dt)
{
    /* 算法不访问时钟；避免零周期/无效输入进入积分和除法。 */
    if (!isfinite(dt) || dt <= 0 || !isfinite(measure) || !isfinite(ref)) {
        pid_clear(pid);
        return 0;
    }
    pid->dt = dt; // 使用任务传入的实际周期，单位秒

    // 保存上次的测量值和误差,计算当前error
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Err = pid->Ref - pid->Measure;

    if (!pid->initialized) {
        pid->Last_Measure = measure;
        pid->Last_Err = pid->Err;
        pid->Pout = pid->Kp * pid->Err;
        pid->initialized = true;
    }
    // 如果在死区外,则计算PID
    if (usr_abs(pid->Err) > pid->DeadBand)
    {
        // 基本的pid计算,使用位置式
        /* 原头文件声明了比例先行开关，但实现中未使用；这里补齐。 */
        if (pid->Improve & PID_Proportional_On_Measurement)
            pid->Pout -= pid->Kp * (pid->Measure - pid->Last_Measure);
        else
            pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->dt;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err) / pid->dt;

        // 梯形积分
        if (pid->Improve & PID_Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        // 变速积分
        if (pid->Improve & PID_ChangingIntegrationRate)
            f_Changing_Integration_Rate(pid);
        // 微分先行
        if (pid->Improve & PID_Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        // 微分滤波器
        if (pid->Improve & PID_DerivativeFilter)
            f_Derivative_Filter(pid);
        // 积分限幅
        if (pid->Improve & PID_Integral_Limit)
            f_Integral_Limit(pid);

        pid->Iout += pid->ITerm;                         // 累加积分
        pid->Output = pid->Pout + pid->Iout + pid->Dout; // 计算输出

        // 输出滤波
        if (pid->Improve & PID_OutputFilter)
            f_Output_Filter(pid);

        // 输出限幅
        f_Output_Limit(pid);
    }
    else // 进入死区, 则清空积分和输出
    {
        pid->Output = 0;
        pid->ITerm = 0;
        pid->Iout = 0; /* 与原注释一致：进入死区确实清积分。 */
        pid->Pout = 0; pid->Dout = 0;
    }

    if (pid->Improve & PID_ErrorHandle) f_PID_ErrorHandle(pid);

    // 保存当前数据,用于下次计算
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;

    return pid->Output;
}

/**
 * @brief 清空一个pid的历史数据
 *
 * @param pid    PID实例
 */
void pid_clear(pid_obj_t *pid)
{
    pid->initialized = false;
    pid->Ref = 0; pid->dt = 0;
    pid->Measure = 0;
    pid->Last_Measure = 0;
    pid->Err=0;
    pid->Last_Err=0;
    pid->Last_ITerm=0;
    pid->Pout=0;
    pid->Iout=0;
    pid->Dout=0;
    pid->ITerm=0;
    pid->Output=0;
    pid->Last_Output=0;
    pid->Last_Dout=0;
    pid->ERRORHandler.error_count=0;
    pid->ERRORHandler.error_type=0;
}
