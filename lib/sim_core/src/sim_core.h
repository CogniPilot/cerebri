#ifndef A7E1DA61_9CBB_4AD6_9749_15CEA1A70318
#define A7E1DA61_9CBB_4AD6_9749_15CEA1A70318

#include "LockingQueue.h"
#include "messages.h"

struct sim_time_t {
    int64_t sec;
    int32_t nsec;
};

extern LockingQueue<msg_accelerometer_t> queue_accelerometer;
extern LockingQueue<msg_actuators_t> queue_actuator;
extern LockingQueue<msg_altimeter_t> queue_altimeter;
extern LockingQueue<msg_gyroscope_t> queue_gyroscope;
extern LockingQueue<msg_magnetometer_t> queue_magnetometer;
extern LockingQueue<msg_navsat_t> queue_navsat;
extern LockingQueue<msg_odometry_t> queue_external_odometry;
extern LockingQueue<msg_rc_input_t> queue_rc_input;
extern LockingQueue<msg_trajectory_t> queue_trajectory;
extern LockingQueue<sim_time_t> queue_sim_time;

#endif /* A7E1DA61_9CBB_4AD6_9749_15CEA1A70318 */
