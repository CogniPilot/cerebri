#ifndef RDD2_ATTITUDE_CONTROL_H_
#define RDD2_ATTITUDE_CONTROL_H_

#include "pid_axis.h"
#include "topic_flatbuffer.h"

struct rdd2_attitude_controller {
	struct rdd2_pid_axis roll;
	struct rdd2_pid_axis pitch;
};

void rdd2_attitude_controller_init(struct rdd2_attitude_controller *controller);
void rdd2_attitude_controller_reset(struct rdd2_attitude_controller *controller);
void rdd2_attitude_desired_from_rc(const synapse_topic_RcChannels16_t *rc,
				   const synapse_topic_AttitudeEuler_t *attitude,
				   synapse_topic_AttitudeEuler_t *attitude_desired);
void rdd2_attitude_controller_step(struct rdd2_attitude_controller *controller,
				   const synapse_topic_AttitudeEuler_t *attitude,
				   const synapse_topic_AttitudeEuler_t *attitude_desired,
				   const synapse_topic_RcChannels16_t *rc, float dt,
				   synapse_topic_RateTriplet_t *rate_desired);

#endif
