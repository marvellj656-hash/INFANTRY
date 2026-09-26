#include "ramp.h"
#include <math.h>
void ramp_update_xy(float *x, float *y, float target_x, float target_y,
                    float acceleration, float dt_seconds)
{
    float dx = target_x - *x, dy = target_y - *y;
    float distance = hypotf(dx, dy);
    float step = acceleration * dt_seconds;
    float ratio = distance > step ? step / distance : 1;
    *x += dx * ratio;
    *y += dy * ratio;
}
float ramp_update(float current, float target, float acceleration, float dt_seconds)
{
    float step = acceleration * dt_seconds;
    float delta = target - current;
    if (delta > step) delta = step;
    if (delta < -step) delta = -step;
    return current + delta;
}
