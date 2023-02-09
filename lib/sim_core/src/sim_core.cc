#include <zephyr/kernel.h>
#include <iostream>
#include <cmath>
#include <atomic>

#include "sim_core.h"

#include "channels.h"

static const int clock_check_rate = 1000; // Hz
int64_t connect_time = 0;

LockingQueue<msg_accelerometer_t> queue_accelerometer{};
LockingQueue<msg_actuators_t> queue_actuator{};
LockingQueue<msg_altimeter_t> queue_altimeter{};
LockingQueue<msg_gyroscope_t> queue_gyroscope{};
LockingQueue<msg_magnetometer_t> queue_magnetometer{};
LockingQueue<msg_navsat_t> queue_navsat{};
LockingQueue<msg_odometry_t> queue_external_odometry{};
LockingQueue<msg_rc_input_t> queue_rc_input{};
LockingQueue<msg_trajectory_t> queue_trajectory{};
LockingQueue<sim_time_t> queue_sim_time{};

#define PUB_SIM_MESSAGES(TOPIC, MSG) \
{ \
    msg_ ## MSG ## _t data; \
    while (queue_##TOPIC.tryPop(data)) { \
        zbus_chan_pub(&chan_##TOPIC,  &data, K_FOREVER); \
    } \
}

void sim_actuator_callback(const struct zbus_channel *chan)
{
    msg_actuators_t msg = *(const msg_actuators_t *)zbus_chan_const_msg(chan);
    queue_actuator.push(msg);
};
ZBUS_LISTENER_DEFINE(listener_sim_actuators, sim_actuator_callback);

void thread_sim_core_entry_point(void *p1, void *p2, void *p3) {
    printf("waiting for simulation connection\n");
    while (true)
    {
        int64_t uptime = k_uptime_get();
        int64_t sec = uptime/1.0e3;
        int32_t nsec = (uptime - sec*1e3)*1e6;

        // get the latest simulation time
        sim_time_t sim_time;
        while (queue_sim_time.tryPop(sim_time)) {
        }

        if (sim_time.sec == 0 && sim_time.nsec == 0) {
            // first run, need to fast forward clock
        }

        // fast forward zephyClock time to match sim
        int64_t delta_sec = sim_time.sec - sec;
        int32_t delta_nsec = sim_time.nsec - nsec;
        int32_t wait = delta_sec*1e6 + delta_nsec/1e3;

        // advance the zephyr clock
        if (wait > 0) {
            //printf("wait: %d\n", wait);
            if (connect_time == 0) {
                connect_time = k_uptime_get();
                printf("simulator connected\n");
            }
            k_usleep(wait);
        }

        // publish all messages queued
        PUB_SIM_MESSAGES(accelerometer, accelerometer);
        PUB_SIM_MESSAGES(gyroscope, gyroscope);
        PUB_SIM_MESSAGES(magnetometer, magnetometer);
        PUB_SIM_MESSAGES(altimeter, altimeter);
        PUB_SIM_MESSAGES(navsat, navsat);
        PUB_SIM_MESSAGES(trajectory, trajectory);
        PUB_SIM_MESSAGES(rc_input, rc_input);
        PUB_SIM_MESSAGES(external_odometry, odometry);

        // allow other threads to run
        k_yield();

        // this sleeps the sim to save cpu load
        struct timespec tsleep, trem;
        tsleep.tv_sec = 0;
        tsleep.tv_nsec = 1e9/clock_check_rate;
        nanosleep(&tsleep, &trem);
    }
}

K_THREAD_DEFINE(sim_core, 1024, thread_sim_core_entry_point, NULL, NULL, NULL, 7, 0, 0);
