#include "referee_ui/referee_ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "UI check line %d: %s\n", __LINE__, #c); exit(1); } } while (0)
/* 用独立逐位读取来查协议字段，避免跟编码器用同一种拼接方法自证。 */
static uint32_t bits(const uint8_t *data, unsigned start, unsigned length)
{
    uint32_t result = 0u;
    for (unsigned i = 0; i < length; ++i)
        if (data[(start + i) / 8u] & (1u << ((start + i) % 8u))) result |= 1u << i;
    return result;
}
int main(void)
{
    RefereeUiModel model;
    referee_ui_reset(&model);
    RefereeUiInput input = {.commands_fresh = true, .friction = true, .spin = false,
        .auto_aim = true, .heat_fresh = true, .heat = 321u, .heat_limit = 500u};
    RefereeUiPacket packet, retry;
    CHECK(referee_ui_client_id(3u) == 0x103u && referee_ui_client_id(103u) == 0x167u);
    CHECK(referee_ui_client_id(0u) == 0u && referee_ui_client_id(255u) == 0u);
    const unsigned x[REF_UI_COUNT] = {430,430,430,1320,935,960,960,529,1391};
    const unsigned y[REF_UI_COUNT] = {850,775,700,720,540,570,540,102,102};
    for (unsigned i = 0; i < REF_UI_COUNT; ++i)
    {
        CHECK(referee_ui_prepare(&model, &input, &packet));
        CHECK(packet.element == i && bits(packet.bytes, 24u, 3u) == 1u); /* ADD */
        CHECK(bits(packet.bytes, 66u, 11u) == x[i]);
        CHECK(bits(packet.bytes, 77u, 11u) == y[i]);
        CHECK(packet.bytes[0] == 'H' && packet.bytes[1] == 'N' && packet.bytes[2] == '0' + i);
        CHECK(packet.length == (i <= REF_UI_HEAT ? 45u : 15u));
        CHECK(packet.data_command == (i <= REF_UI_HEAT ? 0x0110u : 0x0101u));
        if (i == REF_UI_FRICTION || i == REF_UI_AIM) CHECK(bits(packet.bytes, 34u, 4u) == 2u);
        if (i == REF_UI_SPIN) CHECK(bits(packet.bytes, 34u, 4u) == 4u);
        if (i == REF_UI_HEAT)
        {
            CHECK(memcmp(packet.bytes + 15u, "Heat:321/500", 12u) == 0);
            CHECK(packet.bytes[27] == ' ' && bits(packet.bytes, 47u, 9u) == 16u);
        }
        if (i == REF_UI_AIM_CIRCLE)
        {
            CHECK(bits(packet.bytes, 27u, 3u) == 2u && bits(packet.bytes, 88u, 10u) == 250u);
        }
        CHECK(referee_ui_prepare(&model, &input, &retry));
        CHECK(memcmp(&packet, &retry, sizeof(packet)) == 0); /* 未commit就像忙失败重试 */
        referee_ui_commit(&model, &packet);
    }
    CHECK(model.next_element == 0u && model.added_mask == 0x1ffu);
    CHECK(referee_ui_prepare(&model, &input, &packet) && bits(packet.bytes, 24u, 3u) == 2u);
    input.commands_fresh = false;
    CHECK(referee_ui_prepare(&model, &input, &packet) && bits(packet.bytes, 34u, 4u) == 4u);
    model.next_element = REF_UI_HEAT;
    input.heat_fresh = false;
    CHECK(referee_ui_prepare(&model, &input, &packet));
    CHECK(memcmp(packet.bytes + 15u, "Heat:---/---", 12u) == 0);
    input.heat_fresh = true; input.heat = 65535u; input.heat_limit = 65535u;
    CHECK(referee_ui_prepare(&model, &input, &packet));
    CHECK(memcmp(packet.bytes + 15u, "Heat:65535/65535", 16u) == 0);
    referee_ui_reset(&model);
    CHECK(referee_ui_prepare(&model, &input, &packet) && bits(packet.bytes, 24u, 3u) == 1u);
    CHECK(!referee_ui_prepare(NULL, &input, &packet));
    puts("referee_ui: 9 layouts, bit positions, ADD/MODIFY, retry, stale state and heat text passed");
    return 0;
}
