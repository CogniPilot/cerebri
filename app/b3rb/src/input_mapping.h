#ifndef CEREBRI_RDD2_INPUT_MAPPING_H_
#define CEREBRI_RDD2_INPUT_MAPPING_H_

#include <stdbool.h>
#include <synapse_pb/input.pb.h>

enum {
    CH_RIGHT_STICK_RIGHT = 0,
    CH_RIGHT_STICK_UP = 1,
    CH_LEFT_STICK_UP = 2,
    CH_LEFT_STICK_RIGHT = 3,
    CH_SWA_DOWN = 4,
    CH_SWB_DOWN = 5,
    CH_SWC_DOWN = 6,
    CH_SWD_DOWN = 7,
    CH_VRA_CW = 8,
    CH_VRB_CCW = 9,
};

enum {
    SWITCH_POS_UP = 0,
    SWITCH_POS_MIDDLE = 1,
    SWITCH_POS_DOWN = 2,
};

int two_position_switch(float val);
int three_position_switch(float val);

struct input_request {
    bool arm;
    bool disarm;
    bool topic_source_input;
    bool topic_source_ethernet;
    bool mode_actuators;
    bool mode_angular_velocity;
    bool mode_attitude;
    bool mode_attitude_rate;
    bool mode_velocity;
    bool mode_bezier;
    bool mode_calibration;
    bool lights_on;
};

void input_request_compute(struct input_request* req, const synapse_pb_Input* input);

#endif // CEREBRI_RDD2_INPUT_MAPPING_H_

// vi: ts=4 sw=4 et
