#ifndef MOTION_COMMAND_H
#define MOTION_COMMAND_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { float vx, vy; } PlanarVelocity;
PlanarVelocity motion_target(int16_t ch1, int16_t ch2, bool enabled);
PlanarVelocity motion_step(PlanarVelocity current, PlanarVelocity target,
                           float dt_seconds);
#endif
