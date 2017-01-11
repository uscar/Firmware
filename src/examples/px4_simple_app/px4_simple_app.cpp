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
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>

extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Starting testing application...");
  uORB::Subscription<home_position_s> home_pos_sub(ORB_ID(home_position), -1);
  uORB::Publication<position_setpoint_triplet_s> pos_setpoint_pub(
      ORB_ID(position_setpoint_triplet), -1);
  uORB::Publication<vehicle_control_mode_s> control_mode_pub(
      ORB_ID(vehicle_control_mode), -1);

  uORB::Publication<actuator_armed_s> actuator_armed_pub(
      ORB_ID(actuator_armed), -1);
  actuator_armed_pub.get().armed = true;
  actuator_armed_pub.update();

  control_mode_pub.get().flag_control_offboard_enabled = true;
  control_mode_pub.get().flag_control_position_enabled = true;
  control_mode_pub.get().flag_control_altitude_enabled = true;
  control_mode_pub.update();

  home_position_s home = home_pos_sub.get();
  pos_setpoint_pub.get().previous.x = home.x;
  pos_setpoint_pub.get().previous.y = home.y;
  pos_setpoint_pub.get().previous.z = home.z;

   PX4_INFO("home.x: %f", (double) home.x);
   PX4_INFO("home.y: %f", (double) home.y);
   PX4_INFO("home.z: %f", (double) home.z);
  
  pos_setpoint_pub.get().current.x = 55;
  pos_setpoint_pub.get().current.y = 55;
  pos_setpoint_pub.get().current.z = 55;
  // Hack for NaN since we don't have access to std::numeric_limits
  pos_setpoint_pub.get().current.yaw = sqrt(-1);
  pos_setpoint_pub.get().current.yaw_valid = 55;

  pos_setpoint_pub.get().current.type = position_setpoint_s::SETPOINT_TYPE_OFFBOARD;
  pos_setpoint_pub.get().current.valid = true;
  pos_setpoint_pub.get().current.position_valid = true;

  pos_setpoint_pub.update();

	return 0;
}
