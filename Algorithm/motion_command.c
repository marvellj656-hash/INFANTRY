#include "motion_command.h"
#include <math.h>
/* First bring-up limits; mm/s and mm/s^2, not motor RPM. */
#define MAX_SPEED 2000.0f
#define MAX_ACCELERATION 500.0f
static float axis(int16_t value)
{
    if (value > 660) value = 660;
    if (value < -660) value = -660;
    if (value > 5) return (value - 5) / 655.0f;
    if (value < -5) return (value + 5) / 655.0f;
    return 0.0f;
}
PlanarVelocity motion_target(int16_t ch1, int16_t ch2, bool enabled)
{
    PlanarVelocity v = {0};
    if (!enabled) return v;
    v.vx = axis(ch1); v.vy = axis(ch2);
    float magnitude = sqrtf(v.vx * v.vx + v.vy * v.vy);
    if (magnitude > 1.0f) { v.vx /= magnitude; v.vy /= magnitude; }
    v.vx *= MAX_SPEED; v.vy *= MAX_SPEED;
    return v;
}
PlanarVelocity motion_step(PlanarVelocity current, PlanarVelocity target,
                           float dt_seconds)
{
    if (!isfinite(dt_seconds) || dt_seconds < 0 || dt_seconds > 0.05f)
        return (PlanarVelocity){0};
    float dx = target.vx - current.vx, dy = target.vy - current.vy;
    float distance = sqrtf(dx * dx + dy * dy);
    float step = MAX_ACCELERATION * dt_seconds;
    if (distance <= step || distance == 0) return target;
    current.vx += dx * step / distance;
    current.vy += dy * step / distance;
    return current;
}
