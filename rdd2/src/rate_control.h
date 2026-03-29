#ifndef RDD2_RATE_CONTROL_H_
#define RDD2_RATE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "generated/PIDAxis.h"
#include "topic_flatbuffer.h"

#define RDD2_CONTROL_PERIOD_US          1250
#define RDD2_RC_STALE_TIMEOUT_MS        100
#define RDD2_THROTTLE_ARM_MAX          1050
#define RDD2_PID_INTEGRATE_THROTTLE_MIN 0.02f
#define RDD2_ARM_CHANNEL_INDEX             4
#define RDD2_ROLL_CHANNEL_INDEX            0
#define RDD2_PITCH_CHANNEL_INDEX           1
#define RDD2_THROTTLE_CHANNEL_INDEX        2
#define RDD2_YAW_CHANNEL_INDEX             3
#define RDD2_MAX_ROLL_PITCH_RATE_RAD_S  6.0f
#define RDD2_MAX_YAW_RATE_RAD_S         3.5f

struct rdd2_rate_controller {
	PIDAxis_t roll;
	PIDAxis_t pitch;
	PIDAxis_t yaw;
};

void rdd2_rate_controller_init(struct rdd2_rate_controller *controller);
void rdd2_rate_controller_reset(struct rdd2_rate_controller *controller);
bool rdd2_rate_arm_switch_high(const synapse_topic_RcChannels16_t *rc);
int32_t rdd2_rate_throttle_us(const synapse_topic_RcChannels16_t *rc);
float rdd2_rate_throttle_input_from_rc(const synapse_topic_RcChannels16_t *rc);
float rdd2_rate_throttle_command(float throttle_input, bool armed);
void rdd2_rate_desired_from_rc(const synapse_topic_RcChannels16_t *rc,
				  synapse_topic_RateTriplet_t *rate_desired);
void rdd2_rate_controller_step(struct rdd2_rate_controller *controller,
				  const synapse_topic_RateTriplet_t *rate_desired,
				  const synapse_topic_Vec3f_t *gyro, float dt,
				  bool integrate,
				  synapse_topic_RateTriplet_t *rate_cmd);
void rdd2_mix_quad_x(float throttle, const synapse_topic_RateTriplet_t *rate_cmd,
			synapse_topic_MotorValues4f_t *motors);

#endif
