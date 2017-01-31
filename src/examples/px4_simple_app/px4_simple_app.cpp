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
#include <px4_time.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include "systemlib/param/param.h"
#include "drivers/drv_pwm_output.h"

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
#include <uORB/topics/optical_flow.h>

extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[]);

int px4_simple_app_main(int argc, char *argv[])
{
	PX4_INFO("Starting testing application...");
  uORB::Subscription<optical_flow_s> flow_sub(ORB_ID(optical_flow));
  uORB::Subscription<home_position_s> home_pos_sub(ORB_ID(home_position));
  uORB::Subscription<sensor_combined_s> sensor_sub(ORB_ID(sensor_combined));

  uORB::Subscription<vehicle_attitude_setpoint_s> vehicle_setpoint_sub(ORB_ID(vehicle_attitude_setpoint));

  uORB::Publication<vehicle_attitude_setpoint_s> vehicle_setpoint_pub(
      ORB_ID(vehicle_attitude_setpoint), 0);

  uORB::Publication<position_setpoint_triplet_s> pos_setpoint_pub(
      ORB_ID(position_setpoint_triplet), -1);
  uORB::Publication<vehicle_control_mode_s> control_mode_pub(
      ORB_ID(vehicle_control_mode), -1);

  uORB::Publication<actuator_armed_s> actuator_armed_pub(
      ORB_ID(actuator_armed), -1);

  
  // actuator_armed_pub.get().ready_to_arm = true;
  // actuator_armed_pub.get().armed = true;
  // actuator_armed_pub.update();

  // control_mode_pub.get().flag_control_offboard_enabled = true;
  // control_mode_pub.get().flag_control_position_enabled = true;
  // control_mode_pub.get().flag_control_altitude_enabled = true;
  // control_mode_pub.update();

  // pos_setpoint_pub.get().previous.x = home.x;
  // pos_setpoint_pub.get().previous.y = home.y;
  // pos_setpoint_pub.get().previous.z = home.z;
  
  // pos_setpoint_pub.get().current.x = 55;
  // pos_setpoint_pub.get().current.y = 55;
  // pos_setpoint_pub.get().current.z = 55;
  // // Hack for NaN since we don't have access to std::numeric_limits
  // pos_setpoint_pub.get().current.yaw = sqrt(-1);
  // pos_setpoint_pub.get().current.yaw_valid = 55;

  // pos_setpoint_pub.get().current.type = position_setpoint_s::SETPOINT_TYPE_OFFBOARD;
  // pos_setpoint_pub.get().current.valid = true;
  // pos_setpoint_pub.get().current.position_valid = true;

  // pos_setpoint_pub.update();

  double thrust = 0.0;
  while (true) {
    vehicle_setpoint_pub.get().roll_body = 0.0;
    vehicle_setpoint_pub.get().pitch_body = 0.0;
    vehicle_setpoint_pub.get().yaw_body = 0.0;
    vehicle_setpoint_pub.get().R_valid = false;
    vehicle_setpoint_pub.get().yaw_sp_move_rate = 0.0;
    vehicle_setpoint_pub.get().thrust = (float) thrust;
    vehicle_setpoint_pub.get().roll_reset_integral = false;
    vehicle_setpoint_pub.get().pitch_reset_integral = false;
    vehicle_setpoint_pub.get().yaw_reset_integral = false;
    vehicle_setpoint_pub.get().fw_control_yaw = false;
    vehicle_setpoint_pub.get().disable_mc_yaw_control = false;
    vehicle_setpoint_pub.get().apply_flaps = false;
    vehicle_setpoint_pub.update();

    thrust += 0.0001;

    flow_sub.update();
    auto flow = flow_sub.get();

    sensor_sub.update();
    auto sensors = sensor_sub.get();

    if (flow.ground_distance_m > 1.0f || sensors.baro_alt_meter > 1.0f) {
      PX4_INFO("Above 1.0m. Setting attitude to land.");
      vehicle_setpoint_pub.get().roll_body = 0.0f;
      vehicle_setpoint_pub.get().pitch_body = 0.0f;
      vehicle_setpoint_pub.get().yaw_body = 0.0f;
      vehicle_setpoint_pub.get().yaw_sp_move_rate = 0.0f;
      vehicle_setpoint_pub.get().thrust = 0.0f;
      vehicle_setpoint_pub.get().fw_control_yaw = false;
      vehicle_setpoint_pub.get().disable_mc_yaw_control = false;
      vehicle_setpoint_pub.get().apply_flaps = false;
      vehicle_setpoint_pub.update();

      actuator_armed_pub.get().ready_to_arm = false;
      actuator_armed_pub.get().armed = false;
      actuator_armed_pub.update();

      break;
    }

    vehicle_setpoint_sub.update();
    double r = vehicle_setpoint_sub.get().roll_body;
    double p = vehicle_setpoint_sub.get().pitch_body;
    double y = vehicle_setpoint_sub.get().yaw_body;
    double t = vehicle_setpoint_sub.get().thrust;
    PX4_INFO("Roll: %8.4f\tPitch: %8.4f\tYaw: %8.4f\tThrust: %8.4f\t", r, p, y, t);

    usleep(100000);
  }
  
	return 0;
}
