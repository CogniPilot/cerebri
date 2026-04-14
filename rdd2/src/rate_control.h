#ifndef RDD2_RATE_CONTROL_H_
#define RDD2_RATE_CONTROL_H_

#include <stdbool.h>
#include <stdint.h>

#include "pid_axis.h"
#include "topic_flatbuffer.h"

#define RDD2_CONTROL_RATE_HZ              1600U
#define RDD2_CONTROL_PERIOD_NS            625000ULL
#define RDD2_CONTROL_DT_S                 (1.0f / 1600.0f)
#define RDD2_ATTITUDE_AUTOLEVEL_RATE_HZ   200U
#define RDD2_ATTITUDE_BACKGROUND_RATE_HZ  100U
#define RDD2_FLIGHT_STATE_PUBLISH_RATE_HZ 200U
#define RDD2_ATTITUDE_AUTOLEVEL_DIV       (RDD2_CONTROL_RATE_HZ / RDD2_ATTITUDE_AUTOLEVEL_RATE_HZ)
#define RDD2_ATTITUDE_BACKGROUND_DIV      (RDD2_CONTROL_RATE_HZ / RDD2_ATTITUDE_BACKGROUND_RATE_HZ)
#define RDD2_FLIGHT_STATE_PUBLISH_DIV     (RDD2_CONTROL_RATE_HZ / RDD2_FLIGHT_STATE_PUBLISH_RATE_HZ)
#define RDD2_RC_STALE_TIMEOUT_MS          100
#define RDD2_THROTTLE_ARM_MAX             1050
#define RDD2_PID_INTEGRATE_THROTTLE_MIN   0.02f
#define RDD2_ARM_CHANNEL_INDEX            4
#define RDD2_ROLL_CHANNEL_INDEX           0
#define RDD2_PITCH_CHANNEL_INDEX          1
#define RDD2_THROTTLE_CHANNEL_INDEX       2
#define RDD2_YAW_CHANNEL_INDEX            3
#define RDD2_MAX_ROLL_PITCH_RATE_RAD_S    6.0f
#define RDD2_MAX_YAW_RATE_RAD_S           3.5f

_Static_assert((RDD2_CONTROL_RATE_HZ % RDD2_ATTITUDE_AUTOLEVEL_RATE_HZ) == 0U,
	       "auto-level attitude loop rate must divide the control loop rate");
_Static_assert((RDD2_CONTROL_RATE_HZ % RDD2_ATTITUDE_BACKGROUND_RATE_HZ) == 0U,
	       "background attitude loop rate must divide the control loop rate");
_Static_assert((RDD2_CONTROL_RATE_HZ % RDD2_FLIGHT_STATE_PUBLISH_RATE_HZ) == 0U,
	       "flight-state publish rate must divide the control loop rate");

struct rdd2_rate_controller {
	struct rdd2_pid_axis roll;
	struct rdd2_pid_axis pitch;
	struct rdd2_pid_axis yaw;
};

void rdd2_rate_controller_init(struct rdd2_rate_controller *controller);
void rdd2_rate_controller_reset(struct rdd2_rate_controller *controller);
bool rdd2_rate_arm_switch_high(const synapse_topic_RcChannels16_t *rc);
int32_t rdd2_rate_throttle_us(const synapse_topic_RcChannels16_t *rc);
float rdd2_rate_throttle_input_from_rc(const synapse_topic_RcChannels16_t *rc);
float rdd2_rate_throttle_command(float throttle_input, bool armed);
float rdd2_rate_yaw_desired_from_rc(const synapse_topic_RcChannels16_t *rc);
void rdd2_rate_desired_from_rc(const synapse_topic_RcChannels16_t *rc,
			       synapse_topic_RateTriplet_t *rate_desired);
void rdd2_rate_controller_step(struct rdd2_rate_controller *controller,
			       const synapse_topic_RateTriplet_t *rate_desired,
			       const synapse_topic_Vec3f_t *gyro, float dt, bool integrate,
			       synapse_topic_RateTriplet_t *rate_cmd);
void rdd2_mix_quad_x(float throttle, const synapse_topic_RateTriplet_t *rate_cmd,
		     synapse_topic_MotorValues4f_t *motors);

#endif
