/*
 * Copyright CogniPilot Foundation 2023
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <math.h>

#include <stdio.h>

#include <synapse_zbus/channels.h>


void control_entry_point(void *p1, void *p2, void *p3) {
    while (true) {
        // send data to motors
        Actuators msg = Actuators_init_zero;
        msg.has_header = true;
        strncpy(msg.header.frame_id, "test", 16);
        msg.header.has_stamp = true;
        msg.header.seq = 0;
        msg.header.stamp.nanos = 0;
        msg.header.stamp.seconds = 0;
        msg.normalized_count = 0;
        msg.position_count = 0;
        for (int i=0; i<4; i++) {
            msg.velocity[i] = 1.0;
        }
        msg.velocity_count = 4;
        zbus_chan_pub(&chan_out_actuators, &msg, K_NO_WAIT);
        
        // sleep to set control rate at 10 Hz
        k_usleep(1e6/10);
    }
}

K_THREAD_DEFINE(control_thread, 500,
                control_entry_point, NULL, NULL, NULL,
                -1, 0, 0);

/* vi: ts=4 sw=4 et */


