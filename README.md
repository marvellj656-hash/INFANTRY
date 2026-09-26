# FreeRTOS 机器人框架

第一次阅读从这里开始：[从一个动作读懂框架](Docs/learning_start.md)。当前排查版本：[OMNI9_26](Docs/versions/OMNI9_26.md)。

本次上车入口：[遥控器说明图与排查路线](Docs/bringup_guide.md)、[LinkScope 快捷分组](Docs/LinkScope/README.md)。

当前入口：[架构与开发指南](Docs/architecture.md)、[任务接口](Docs/task_interfaces.md)、[验证与迁移状态](Docs/migration_status.md)。

# **OMNI_freeRTOS_port---Framework**

这版函数命名规律：模块＋动作＋对象
工程通过分层来实现(the whole program is composed with)：Bsp->Modules->App(algorithem)
层次          	回答的问题
App	            这辆车现在应该做什么？
Modules	        遥控器、电机这些设备怎么使用？
Algorithm	    给定输入，怎么算出结果？
Bsp	            字节或报文怎么经过这块板子的外设收发？