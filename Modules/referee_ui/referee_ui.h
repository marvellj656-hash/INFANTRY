#ifndef MODULE_REFEREE_UI_H
#define MODULE_REFEREE_UI_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    REF_UI_FRICTION, REF_UI_SPIN, REF_UI_AIM, REF_UI_HEAT,
    REF_UI_CROSS_H, REF_UI_CROSS_V, REF_UI_AIM_CIRCLE,
    REF_UI_CAR_LEFT, REF_UI_CAR_RIGHT, REF_UI_COUNT
} RefereeUiElement;
typedef struct
{
    bool commands_fresh, friction, spin, auto_aim, heat_fresh;
    uint16_t heat, heat_limit;
} RefereeUiInput;
typedef struct
{
    uint16_t added_mask;
    uint8_t next_element;
} RefereeUiModel;
typedef struct
{
    uint8_t bytes[45]; /* 最长是15字节图形描述+30字节文本。 */
    uint16_t data_command;
    uint8_t length, element;
} RefereeUiPacket;

/* 纯绘图数据模块。不直接读任务、不碰UART、不用编译器位域。 */
void referee_ui_reset(RefereeUiModel *model);
bool referee_ui_prepare(const RefereeUiModel *model, const RefereeUiInput *input,
                        RefereeUiPacket *out);
/* 只有成功排入发送队列后才前进；busy时下一圈仍重试同一个元素。 */
void referee_ui_commit(RefereeUiModel *model, const RefereeUiPacket *packet);
uint16_t referee_ui_client_id(uint8_t robot_id);
#endif
