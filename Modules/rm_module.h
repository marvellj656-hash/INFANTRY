#ifndef OMNI_JPL_RM_MODULE_H
#define OMNI_JPL_RM_MODULE_H

/* 任务可从这里包含常用模块；只汇总声明，不创建对象或启动硬件。
 * 实际登记用 robot.c 的 DjiMotorConfig。 */
#include "motor/dji_motor.h"
#include "remote/remote.h"
#include "imu/bmi088.h"

#endif
