#include "referee_ui.h"
#include <stdio.h>
#include <string.h>

enum { UI_ADD = 1u, UI_MODIFY = 2u, UI_LINE = 0u, UI_CIRCLE = 2u, UI_TEXT = 7u };
enum { UI_TEAM_COLOR = 0u, UI_GREEN = 2u, UI_ORANGE = 3u, UI_PURPLE = 4u, UI_CYAN = 6u };
typedef struct
{
    uint8_t type, layer, color;
    uint16_t width, x, y, end_x, end_y, radius, font_size, text_length;
} Graphic;

static void write_u32_le(uint8_t *out, uint32_t word)
{
    for (unsigned byte = 0; byte < 4u; ++byte)
        out[byte] = (uint8_t)(word >> (8u * byte));
}
static void encode_graphic(uint8_t *out, unsigned element, unsigned operation, const Graphic *g)
{
    /* 3字节名字是客户屏幕认出“同一个图形”的身份证。重画和修改要一直使用同名。 */
    out[0] = 'H'; out[1] = 'N'; out[2] = (uint8_t)('0' + element);
    /* 用移位明说每一位的位置。换编译器/CPU也不依赖C位域的排列顺序。 */
    uint32_t first = operation | ((uint32_t)g->type << 3u) |
        ((uint32_t)g->layer << 6u) | ((uint32_t)g->color << 10u) |
        ((uint32_t)g->font_size << 14u) | ((uint32_t)g->text_length << 23u);
    uint32_t second = g->width | ((uint32_t)g->x << 10u) | ((uint32_t)g->y << 21u);
    uint32_t third = g->radius | ((uint32_t)g->end_x << 10u) | ((uint32_t)g->end_y << 21u);
    write_u32_le(out + 3u, first);
    write_u32_le(out + 7u, second);
    write_u32_le(out + 11u, third);
}
void referee_ui_reset(RefereeUiModel *model)
{
    if (model) memset(model, 0, sizeof(*model));
}
uint16_t referee_ui_client_id(uint8_t robot_id)
{
    /* 旧工程可能离线时硬写红3/蓝3，新模块必须先收到自己的真实身份。 */
    if ((robot_id >= 1u && robot_id <= 6u) || (robot_id >= 101u && robot_id <= 106u))
        return 0x0100u + robot_id;
    return 0u;
}
bool referee_ui_prepare(const RefereeUiModel *model, const RefereeUiInput *input,
                        RefereeUiPacket *out)
{
    if (!model || !input || !out || model->next_element >= REF_UI_COUNT)
        return false;
    memset(out, 0, sizeof(*out));
    const unsigned element = model->next_element;
    unsigned operation = (model->added_mask & (1u << element)) ? UI_MODIFY : UI_ADD;
    Graphic g = {.layer = 3u};
    char text[31] = {0};
    if (element <= REF_UI_HEAT)
    {
        g.type = UI_TEXT; g.width = 3u; g.font_size = 35u; g.x = 430u;
        g.y = (uint16_t)(850u - element * 75u);
        g.color = UI_PURPLE;
        bool enabled = false;
        if (element == REF_UI_FRICTION) { memcpy(text, "Frc", 4u); enabled = input->friction; }
        else if (element == REF_UI_SPIN) { memcpy(text, "Rot", 4u); enabled = input->spin; }
        else if (element == REF_UI_AIM) { memcpy(text, "Aim", 4u); enabled = input->auto_aim; }
        else
        {
            g.x = 1320u; g.y = 720u; g.layer = 5u;
            if (input->heat_fresh)
                (void)snprintf(text, sizeof(text), "Heat:%u/%u", (unsigned)input->heat,
                               (unsigned)input->heat_limit);
            else
                memcpy(text, "Heat:---/---", 13u);
            g.color = input->heat_fresh ? UI_GREEN : UI_PURPLE;
        }
        if (element < REF_UI_HEAT && input->commands_fresh && enabled)
            g.color = UI_GREEN;
        /* 热量可能从1000降到0，空格补齐16字符，避免屏幕尾部留下旧的数字。 */
        g.text_length = element == REF_UI_HEAT ? 16u : 3u;
        memset(out->bytes + 15u, ' ', 30u);
        memcpy(out->bytes + 15u, text, strlen(text));
        out->data_command = 0x0110u;
        out->length = 45u;
    }
    else
    {
        out->data_command = 0x0101u;
        out->length = 15u;
        g.type = UI_LINE;
        if (element == REF_UI_CROSS_H)
        {
            g.color = UI_CYAN; g.width = 2u; g.x = 935u; g.y = 540u;
            g.end_x = 985u; g.end_y = 540u;
        }
        else if (element == REF_UI_CROSS_V)
        {
            g.color = UI_CYAN; g.width = 2u; g.x = 960u; g.y = 570u;
            g.end_x = 960u; g.end_y = 180u;
        }
        else if (element == REF_UI_AIM_CIRCLE)
        {
            g.type = UI_CIRCLE; g.color = UI_TEAM_COLOR; g.width = 5u;
            g.x = 960u; g.y = 540u; g.radius = 250u;
        }
        else if (element == REF_UI_CAR_LEFT)
        {
            g.color = UI_ORANGE; g.width = 5u; g.x = 529u; g.y = 102u;
            g.end_x = 783u; g.end_y = 445u;
        }
        else
        {
            g.color = UI_ORANGE; g.width = 5u; g.x = 1391u; g.y = 102u;
            g.end_x = 1137u; g.end_y = 445u;
        }
    }
    out->element = (uint8_t)element;
    encode_graphic(out->bytes, element, operation, &g);
    return true;
}
void referee_ui_commit(RefereeUiModel *model, const RefereeUiPacket *packet)
{
    if (!model || !packet || packet->element >= REF_UI_COUNT ||
        packet->element != model->next_element)
        return;
    model->added_mask |= (uint16_t)(1u << packet->element);
    model->next_element = (uint8_t)((model->next_element + 1u) % REF_UI_COUNT);
}
