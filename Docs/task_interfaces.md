# 开发者接口速查

当前完整说明见 [架构指南](architecture.md)。公开模块 API 不包含 HAL/FreeRTOS 类型。

## 三种对象的职责

- 配置：启动时告诉设备“你是谁、接在哪里”。
- 句柄：找到对象后一直保存，不能自己伪造或解引用。
- 快照：每个周期复制一份测量结果，只属于调用者。

## 常见用法

```c
#include "motor/dji_motor.h"

DjiMotorHandle yaw = dji_motor_find(MOTOR_GIMBAL_YAW);
DjiMotorFeedback feedback;
bool found = dji_motor_read(yaw, &feedback);
bool ready = found && feedback.online && feedback.transport_ready;
// 在任务自己的控制器里计算，最后提交原始输出码：
if (!ready) dji_motor_relax(yaw);
```

有多个电机时用 read_many/set_outputs 一次读写，数组顺序由任务自己声明。
chassis：FL、FR、RL、RR；gimbal：Yaw、Pitch；shoot：左、右、拨弹。
模块内部负责映射到 CAN 位置，应用不拼四字节/八字节控制帧。

调用者遵守：注册仅在启动阶段；读/提交仅任务上下文；一个电机一个指令生产者。
relax 是提交零电流/电压，生效于下一次设备服务，不是物理断电，也不是零速度刹车。
不提供含糊的 enable：正常的每周期输出提交就是控制租期，过期自动失效。
PID 保存在各自 Controller 中，复位不会影响别人的控制器。

## 任务 API

| 接口 | 作用 |
| --- | --- |
| chassis_set_command | 车体速度目标、许可、归零状态、原始接收时间 |
| gimbal_set_command | 手动角速度或自动绝对角、许可、原始接收时间 |
| shoot_set_command | 摩擦轮开关、模式、扳机状态、原始接收时间 |
| remote_get_state | 多读者遥控快照，返回值表示在线 |
| ins_get_state | 姿态快照，返回值表示有效且未过期 |
| gimbal_get_state | 云台状态快照，返回值表示新鲜；另看 centered |

控制计算：chassis_control_update、gimbal_control_update、shoot_control_update。
均接受显式 dt，不自己读取系统时钟，不直接发 CAN。用于测试的输入可以全在电脑上构造。

## 默认操纵映射

图示和完整规则见 [上车指南](bringup_guide.md)。右拨杆上/中/下为停止/正常/小陀螺；左拨杆上/中/下为摩擦关闭/单发/连发。CH1/CH2 只管底盘平移，CH3/CH4 只管云台。背面拨轮正向扳机、负向反转。

`cmd_map_update` 是可在电脑测试的纯映射；`cmd_task` 只读快照和提交命令。

| 新接口 | 作用 |
|---|---|
| `cmd_get_state` | 多读者命令副本，用于 UI；false 表示遥控离线或过期 |
| `transmission_get_state` | NUC 目标角度及新鲜度；返回 aim_valid |
| `referee_get_state` | 裁判只读副本；返回 online，各字段另查 `.fresh` |
| `referee_send_interaction` | 复制交互数据进 FIFO，统一加帧头/CRC并限速发出 |
| `referee_ui_request_redraw` | 请求重新添加客户端图形，不阻塞调用者 |

接收任务是字节队列的唯一消费者；`*_service.h` 中的写入/解析入口属于模块服务，不供控制任务调用。NUC 的相对角在 cmd 入口转换为 gimbal 接口需要的绝对目标角，底层 USB 不认识云台零点。

## 接一个新模块

在 Modules/名字 放驱动与私有状态；在 Task/名字 放 task/control/config。
在 robot.c 注册接线并创建任务。任务读快照、调用纯计算，再通过句柄提交结果。
新 CAN 设备由 motor_task 的分发处转交其模块，不能再消费同一个 CAN 接收队列。
需要多个读者时提供 get_state；需要逐条事件时提供专门队列，不暴露队列句柄。
