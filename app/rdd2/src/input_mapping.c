#include "input_mapping.h"
#include <math.h>

int two_position_switch(float val)
{
    if (val > 0.0f) {
        return SWITCH_POS_DOWN;
    } else {
        return SWITCH_POS_UP;
    }
}

int three_position_switch(float val)
{
    if (val > 0.25f) {
        return SWITCH_POS_DOWN;
    } else if (fabsf(val) <= 0.25f) {
        return SWITCH_POS_MIDDLE;
    } else {
        return SWITCH_POS_UP;
    }
}

void input_request_compute(struct input_request* req, const synapse_msgs_Input* input)
{
    // handle input
    int swa = two_position_switch(input->channel[CH_SWA_DOWN]);
    int swb = three_position_switch(input->channel[CH_SWB_DOWN]);
    int swc = three_position_switch(input->channel[CH_SWC_DOWN]);
    int swd = two_position_switch(input->channel[CH_SWD_DOWN]);

    req->arm = swa == SWITCH_POS_DOWN;
    req->disarm = swa == SWITCH_POS_UP;

    req->topic_source_input = swd == SWITCH_POS_UP;
    req->topic_source_ethernet = swd == SWITCH_POS_DOWN;

    req->mode_attitude = swb == SWITCH_POS_UP && swc == SWITCH_POS_UP;
    req->mode_velocity = swb == SWITCH_POS_UP && swc == SWITCH_POS_MIDDLE;
    req->mode_bezier = swb == SWITCH_POS_UP && swc == SWITCH_POS_DOWN;

    req->mode_attitude_rate = swb == SWITCH_POS_MIDDLE && swc == SWITCH_POS_UP;
    req->mode_calibration = swb == SWITCH_POS_DOWN && swc == SWITCH_POS_DOWN;
}
