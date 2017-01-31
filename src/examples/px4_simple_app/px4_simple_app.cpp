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

#include "systemlib/param/param.h"
#include "drivers/drv_pwm_output.h"

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/position_setpoint.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position.h>

extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Starting testing application...");
  uORB::Subscription<home_position_s> home_pos_sub(ORB_ID(home_position), 100);
  uORB::Subscription<vehicle_local_position_s> local_pos_sub(
      ORB_ID(vehicle_local_position));
  uORB::Subscription<optical_flow_s> flow_sub(ORB_ID(optical_flow));

  // uORB::Publication<position_setpoint_triplet_s> pos_setpoint_pub(
  //     ORB_ID(position_setpoint_triplet));
  uORB::Publication<vehicle_attitude_setpoint_s> att_setpoint_pub(
      ORB_ID(vehicle_attitude_setpoint));
  uORB::Publication<vehicle_control_mode_s> control_mode_pub(
      ORB_ID(vehicle_control_mode));
  uORB::Publication<actuator_armed_s> actuator_armed_pub(
      ORB_ID(actuator_armed));

  actuator_armed_pub.get().ready_to_arm = true;
  actuator_armed_pub.get().armed = true;
  actuator_armed_pub.update();

  // control_mode_pub.get().flag_control_offboard_enabled = true;
  // control_mode_pub.get().flag_control_position_enabled = true;
  // control_mode_pub.get().flag_control_altitude_enabled = true;
  control_mode_pub.get().flag_control_attitude_enabled = true;
  control_mode_pub.update();

  att_setpoint_pub.get().roll_body = 0.0f;
  att_setpoint_pub.get().pitch_body = 0.0f;
  att_setpoint_pub.get().yaw_body = 0.0f;
  att_setpoint_pub.get().yaw_sp_move_rate = 0.0f;
  att_setpoint_pub.get().thrust = .15f;
  att_setpoint_pub.get().fw_control_yaw = false;
  att_setpoint_pub.get().disable_mc_yaw_control = false;
  att_setpoint_pub.get().apply_flaps = false;
  att_setpoint_pub.update();

  // home_pos_sub.update();
  // home_position_s home = home_pos_sub.get();
  // pos_setpoint_pub.get().previous.x = home.x;
  // pos_setpoint_pub.get().previous.y = home.y;
  // pos_setpoint_pub.get().previous.z = home.z;

  // PX4_INFO("home (x, y, z): %.3f, %.3f, %.3f",
  //         (double) home.x, (double) home.y, (double) home.z);
  // PX4_INFO("home (lat, lon): %.3f, %.3f, %.3f",
  //         (double) home.lat, (double) home.lon);

  // pos_setpoint_pub.get().current.x = 0;
  // pos_setpoint_pub.get().current.y = 0;
  // pos_setpoint_pub.get().current.z = 0;

  // pos_setpoint_pub.get().next.x = 1;
  // pos_setpoint_pub.get().next.y = 1;
  // pos_setpoint_pub.get().next.z = 1;
  
  // // Hack for NaN since we don't have access to std::numeric_limits
  // pos_setpoint_pub.get().current.yaw = sqrt(-1);
  // pos_setpoint_pub.get().current.yaw_valid = true;
  // pos_setpoint_pub.get().current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
  // pos_setpoint_pub.get().current.valid = true;
  // pos_setpoint_pub.get().current.position_valid = false;

  // pos_setpoint_pub.get().next.yaw = sqrt(-1);
  // pos_setpoint_pub.get().next.yaw_valid = true;
  // pos_setpoint_pub.get().next.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
  // pos_setpoint_pub.get().next.valid = true;
  // pos_setpoint_pub.get().next.position_valid = true;

  // pos_setpoint_pub.update();

  while (true) {
    local_pos_sub.update();
    auto lp = local_pos_sub.get();
    flow_sub.update();
    auto flow = flow_sub.get();

    if (flow.ground_distance_m > .25) {
      PX4_INFO("Above .25m. Setting attitude to land.");
      att_setpoint_pub.get().roll_body = 0.0f;
      att_setpoint_pub.get().pitch_body = 0.0f;
      att_setpoint_pub.get().yaw_body = 0.0f;
      att_setpoint_pub.get().yaw_sp_move_rate = 0.0f;
      att_setpoint_pub.get().thrust = 0.0f;
      att_setpoint_pub.get().fw_control_yaw = false;
      att_setpoint_pub.get().disable_mc_yaw_control = false;
      att_setpoint_pub.get().apply_flaps = false;
      att_setpoint_pub.update();
    }

    usleep(100000);
  }

	return 0;
}