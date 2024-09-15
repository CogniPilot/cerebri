#include "input_mapping.h"
#include <math.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(melm_fsm);

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

void input_request_compute(struct input_request *req, const synapse_pb_Input *input)
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

	req->mode_actuators = swb == SWITCH_POS_UP && swc == SWITCH_POS_UP;
	req->mode_velocity = swb == SWITCH_POS_UP && swc == SWITCH_POS_MIDDLE;
	req->mode_bezier = swb == SWITCH_POS_UP && swc == SWITCH_POS_DOWN;

	req->mode_calibration = swb == SWITCH_POS_DOWN && swc == SWITCH_POS_DOWN;

	if (input->channel[CH_VRA_CW] > 0.5f && !req->lights_on) {
		req->lights_on = true;
		LOG_INF("request lights on");
	} else if (input->channel[CH_VRA_CW] < -0.5f && req->lights_on) {
		LOG_INF("request lights off");
		req->lights_on = false;
	}
}
