#pragma once

#include <stddef.h>

typedef double real_t;

enum {
    CUBCONTROL_FIXEDWINGOUTERLOOP_Y_LEN = 0,
    CUBCONTROL_FIXEDWINGOUTERLOOP_P_LEN = 322,
    CUBCONTROL_FIXEDWINGOUTERLOOP_STATE_LEN = 0,
    CUBCONTROL_FIXEDWINGOUTERLOOP_DERIVATIVE_LEN = 0,
    CUBCONTROL_FIXEDWINGOUTERLOOP_EVENT_INDICATOR_LEN = 25,
    CUBCONTROL_FIXEDWINGOUTERLOOP_PERIODIC_EVENT_LEN = 1
};

typedef struct {
    real_t time;
    real_t y[1];
    real_t p[322];
    real_t event_indicators[25];
    real_t event_indicators_prev[25];
    real_t next_periodic_event[1];
} CubControl_FixedWingOuterLoop_t;

void startup(CubControl_FixedWingOuterLoop_t *m);
void dostep(CubControl_FixedWingOuterLoop_t *m, real_t dt);
void recalibrate(CubControl_FixedWingOuterLoop_t *m);