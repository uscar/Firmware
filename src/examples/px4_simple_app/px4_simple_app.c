/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/optical_flow.h>

__EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	/* subscribe to sensor_combined topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 10);

    int attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));

    orb_set_interval(attitude_sub_fd, 10);

    int vehicle_position_fd = orb_subscribe(ORB_ID(vehicle_local_position));

    orb_set_interval(vehicle_position_fd, 10);

    int optical_flow_fd = orb_subscribe(ORB_ID(optical_flow));

    orb_set_interval(optical_flow_fd, 10);

     /* advertise attitude topic */
    struct vehicle_attitude_setpoint_s att_set;
    memset(&att_set, 0, sizeof(att_set));
    orb_advert_t att_set_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_set);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
        { .fd = attitude_sub_fd, .events = POLLIN },
        { .fd = vehicle_position_fd, .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;
    double thrust = 0.0;

	for (int i = 0; i < 6000; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct sensor_combined_s raw;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
				//PX4_INFO("Accelerometer:\t%8.4f\t%8.4f\t%8.4f",
				//	 (double)raw.accelerometer_m_s2[0],
				//	 (double)raw.accelerometer_m_s2[1],
				//	 (double)raw.accelerometer_m_s2[2]);

                struct vehicle_attitude_s raw_att;
                orb_copy(ORB_ID(vehicle_attitude), attitude_sub_fd, &raw_att);
                //PX4_INFO("Pitch:%8.4f\tRoll:%8.4f\tYaw%8.4f\t", (double)raw_att.pitch, (double)raw_att.roll, (double)raw_att.yaw);

                struct vehicle_local_position_s veh_l;
                orb_copy(ORB_ID(vehicle_local_position), vehicle_position_fd, &veh_l);
                PX4_INFO("X:%8.4f\tY:%8.4f\tZ:%8.4f", (double)veh_l.x, (double)veh_l.y, (double)veh_l.z);
                PX4_INFO("Xv:%8.4f\tYv:%8.4f\tZv:%8.4f", (double)veh_l.vx, (double)veh_l.vy, (double)veh_l.vz);
                PX4_INFO("XYValid:%d\tZValid:%d", (int)veh_l.xy_valid, (int)veh_l.z_valid);

                struct optical_flow_s flow;
                orb_copy(ORB_ID(optical_flow), optical_flow_fd, &flow);
                PX4_INFO("FlowX:%8.4f\tFlowZ:%8.4f\tFlowQuality:%8.4f", (double)flow.pixel_flow_x_integral, (double)flow.ground_distance_m, (double)flow.quality);

                thrust += 2.0 / 6000;
                att_set.roll_body = 0.0;
                att_set.pitch_body = 0.0;
                att_set.yaw_body = -2.5; //(double) raw_att.yaw;
                att_set.R_valid = false;
                att_set.yaw_sp_move_rate = 0.0;
                att_set.thrust = thrust;
                att_set.roll_reset_integral = false;
                att_set.pitch_reset_integral = false;
                att_set.yaw_reset_integral = false;
                att_set.fw_control_yaw = false;
                att_set.disable_mc_yaw_control = false;
                att_set.apply_flaps = false;
                
    
                orb_publish(ORB_ID(vehicle_attitude_setpoint), att_set_pub, &att_set);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
