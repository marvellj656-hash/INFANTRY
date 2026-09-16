#include "dji_feedback.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(void)
{
    for (int i = 0; i < 4; ++i) assert(dji_feedback_index(1, 0x201 + i) == i);
    assert(dji_feedback_index(1, 0x207) == 4);
    assert(dji_feedback_index(2, 0x208) == 5);
    assert(dji_feedback_index(2, 0x201) == -1);
    assert(dji_feedback_index(1, 0x208) == -1);
    assert(dji_feedback_index(0, 0x201) == -1);
    uint8_t data[8] = {0x16, 0x9c, 0xff, 0x9c, 0x80, 0x00, 55, 0};
    DjiFeedback value = {0};
    assert(dji_feedback_decode(data, 8, &value));
    assert(value.ecd == 5788 && value.speed_rpm == -100);
    assert(value.current_raw == -32768 && value.temperature == 55);
    DjiFeedback before = value;
    assert(!dji_feedback_decode(data, 7, &value));
    assert(!dji_feedback_decode(NULL, 8, &value));
    assert(!dji_feedback_decode(data, 8, NULL));
    data[0] = 0x20; data[1] = 0;
    assert(!dji_feedback_decode(data, 8, &value));
    assert(memcmp(&before, &value, sizeof value) == 0);
    data[0] = 0x1f; data[1] = 0xff;
    data[2] = 0x7f; data[3] = 0xff;
    data[4] = 0xff; data[5] = 0xff;
    assert(dji_feedback_decode(data, 8, &value));
    assert(value.ecd == 8191 && value.speed_rpm == 32767 && value.current_raw == -1);
    puts("PASS: motor CAN routing, big-endian signed feedback, invalid-frame rejection");
    return 0;
}
