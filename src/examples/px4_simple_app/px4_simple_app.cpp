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
#include <drivers/drv_hrt.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <vector>
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
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/rc_channels.h>
#include <systemlib/mavlink_log.h>
extern "C" __EXPORT int posctl_main(int argc, char *argv[]);

static orb_advert_t    mavlink_log_pub = nullptr; ///< mavlink log pub
static bool run_task = false;
static int daemon_task;

static float HEIGHT_OFFSET = -0.4f;

int main_thread(int argc, char* argv[]);
void setQuaternionAttSetpoint(vehicle_attitude_setpoint_s & set, double roll, double pitch, double yaw);
std::vector<position_setpoint_s> GetWaypoints(float, float);

float GetDistance(
    const vehicle_local_position_s& current,
    const position_setpoint_s& target);

std::vector<position_setpoint_s> GetWaypoints(float initialZ, float initialYaw) {
  std::vector<position_setpoint_s> waypoints(1);
  for (auto& sp : waypoints) {
    sp.type = position_setpoint_s::SETPOINT_TYPE_OFFBOARD;
    sp.valid = true;
    sp.position_valid = true;
    sp.alt_valid = true;
    sp.yaw_valid = true;
    sp.yaw = initialYaw;
  }
  waypoints[0].x = 0.0f;
  waypoints[0].y = 0.0f;
  waypoints[0].z = initialZ + HEIGHT_OFFSET;

  // waypoints[1].x = 0.0f;
  // waypoints[1].y = 0.0f;
  // waypoints[1].z = -0.35f;

  // waypoints[2].x = 0.0f;
  // waypoints[2].y = 0.0f;
  // waypoints[2].z = -5.0f;

  // waypoints[3].x = 0.0f;
  // waypoints[3].y = 0.0f;
  // waypoints[3].z = -0.5f;
  return waypoints;
}
float GetDistance(const vehicle_local_position_s& current,
                  const position_setpoint_s& target) {
  float x_diff = target.x - current.x;
  float y_diff = target.y - current.y;
  float z_diff = target.z - current.z;
  
  return sqrtf(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff);
}
int posctl_main(int argc, char *argv[])
{
 if (argc < 2) {
   PX4_LOG("Usage: px4_simple_app <start> | <stop>");
   return 1;
 }
 if (!strcmp(argv[1], "start")) {
   if (!run_task) {
        mavlink_and_console_log_info(&mavlink_log_pub, "Enabling testing application...");
     run_task = true;
     daemon_task = px4_task_spawn_cmd("px4_simple_app",
                SCHED_DEFAULT,
                SCHED_PRIORITY_MAX - 5,
                13500,
                main_thread,
                (argv && argc > 2) ?
                (char *const *) &argv[2] : (char *const *) nullptr);
   }
 } else if (!strcmp(argv[1], "stop")) {
   run_task = false;
 } else {
   PX4_LOG("Usage: px4_simple_app <start> | <stop>");
 }
 return 0;
}
void setQuaternionAttSetpoint(vehicle_attitude_setpoint_s & set, double roll, double pitch, double yaw) {
  double t0 = std::cos(yaw * 0.5);
  double t1 = std::sin(yaw * 0.5);
  double t2 = std::cos(roll * 0.5);
  double t3 = std::sin(roll * 0.5);
  double t4 = std::cos(pitch * 0.5);
  double t5 = std::sin(pitch * 0.5);
  set.q_d[0] = t0 * t2 * t4 + t1 * t3 * t5;
  set.q_d[1] = t0 * t3 * t4 - t1 * t2 * t5;
  set.q_d[2] = t0 * t2 * t5 + t1 * t3 * t4;
  set.q_d[3] = t1 * t2 * t4 - t0 * t3 * t5;
  set.q_d_valid = true;
}
int main_thread(int argc, char *argv[])
{
  constexpr uint64_t DELAY = 5E6L;
  // constexpr uint64_t TIMEOUT = 30E6L;
  // constexpr uint64_t E_TIMEOUT = TIMEOUT + 10E6L;
  usleep(DELAY);
  mavlink_and_console_log_info(&mavlink_log_pub, "Starting testing application...");

  uORB::Subscription<optical_flow_s> flow_sub(ORB_ID(optical_flow));
  uORB::Subscription<home_position_s> home_pos_sub(ORB_ID(home_position));
  uORB::Subscription<sensor_combined_s> sensor_sub(ORB_ID(sensor_combined));
  uORB::Subscription<vehicle_attitude_s> vehicle_attitude_sub(ORB_ID(vehicle_attitude));
  uORB::Subscription<vehicle_attitude_setpoint_s> vehicle_setpoint_sub(ORB_ID(vehicle_attitude_setpoint));
  uORB::Subscription<battery_status_s> battery_sub(ORB_ID(battery_status));
  uORB::Subscription<vehicle_local_position_s> vehicle_local_position_sub(ORB_ID(vehicle_local_position));
  uORB::Subscription<rc_channels_s> rc_channels_sub(ORB_ID(rc_channels));

  // uORB::Publication<vehicle_attitude_setpoint_s> vehicle_setpoint_pub(
  //     ORB_ID(vehicle_attitude_setpoint), 0);
  // uORB::Publication<position_setpoint_triplet_s> pos_setpoint_pub(
  //     ORB_ID(position_setpoint_triplet), 0);
  uORB::Publication<vehicle_control_mode_s> control_mode_pub(
      ORB_ID(vehicle_control_mode), 0);
  uORB::Publication<actuator_armed_s> actuator_armed_pub(
      ORB_ID(actuator_armed), 0);
  actuator_armed_pub.get().ready_to_arm = true;
  actuator_armed_pub.get().armed = true;
  actuator_armed_pub.update();
  control_mode_pub.get().flag_armed = true;
  control_mode_pub.get().flag_control_offboard_enabled = false;
  control_mode_pub.get().flag_control_position_enabled = false;
  control_mode_pub.get().flag_control_altitude_enabled = false;
  control_mode_pub.get().flag_control_attitude_enabled = true;
  control_mode_pub.get().flag_control_rattitude_enabled = true;
  control_mode_pub.get().flag_control_velocity_enabled = false;
  control_mode_pub.get().flag_control_climb_rate_enabled = false;
  control_mode_pub.get().flag_control_rates_enabled = true;
  control_mode_pub.get().flag_control_manual_enabled = true;
  control_mode_pub.update();
  vehicle_setpoint_sub.update();
  sensor_sub.update();
  flow_sub.update();
  battery_sub.update();
  vehicle_local_position_sub.update();
  rc_channels_sub.update();
  usleep(1000000);

  // initial z reading
  auto lpos = vehicle_local_position_sub.get();
  float initialZ = lpos.z;

  // Waypoints to visit after first takeoff
  std::vector<position_setpoint_s> waypoints = GetWaypoints(initialZ, lpos.yaw);
  // pos_setpoint_pub.get().current.type = position_setpoint_s::SETPOINT_TYPE_OFFBOARD;
  // pos_setpoint_pub.get().current.valid = true;
  // pos_setpoint_pub.get().current.position_valid = true;
  // pos_setpoint_pub.get().current.alt_valid = true;
  // pos_setpoint_pub.get().current.yaw_valid = true;
  // pos_setpoint_pub.get().current.x = 0.0f;
  // pos_setpoint_pub.get().current.y = 0.0f;
  // pos_setpoint_pub.get().current.z = initialZ + HEIGHT_OFFSET; // set z setpoint .25 higher than initial z
  // pos_setpoint_pub.get().current.yaw = lpos.yaw;
  // pos_setpoint_pub.get().current.disable_mc_yaw_control = true;
  // pos_setpoint_pub.update();
  // constexpr float TOLERANCE_RADIUS = 0.1f;
  // constexpr float SHUTOFF_H = 1.0f;
  // constexpr float LAND_THRESHOLD = -0.075f;
  // int waypoint_index = 0;
  // position_setpoint_s current_setpoint = pos_setpoint_pub.get().current;

  usleep(5E5L);

  // uint64_t start_time = hrt_absolute_time();
  while (run_task) {
    // HEIGHT_OFFSET -= 0.001f;

    rc_channels_sub.update();
    vehicle_attitude_sub.update();
    vehicle_setpoint_sub.update();
    sensor_sub.update();
    flow_sub.update();
    battery_sub.update();
    vehicle_local_position_sub.update();
    lpos = vehicle_local_position_sub.get();
    // float dist = GetDistance(lpos, current_setpoint);
    mavlink_and_console_log_info(&mavlink_log_pub,"(xy, z, fqual) valid: %d, %d, %d",
             lpos.xy_valid, lpos.z_valid, flow_sub.get().quality);
    mavlink_and_console_log_info(&mavlink_log_pub,"(x, y, z): %6.4f, %6.4f, %6.4f",
            (double) lpos.x, (double) lpos.y, (double) lpos.z);
    // mavlink_and_console_log_info(&mavlink_log_pub,"(sx, sy, sz): %6.4f, %6.4f, %6.4f",
    //         (double) current_setpoint.x, (double) current_setpoint.y, (double) current_setpoint.z);
    // mavlink_and_console_log_info(&mavlink_log_pub,"Distance From Waypoint: %6.4f", (double) dist);

    auto rc = rc_channels_sub.get();
    mavlink_and_console_log_info(&mavlink_log_pub,"SS: %6.4f C1: %6.4f C2: %6.4f C3: %6.4f C4: %6.4f C5: %6.4f ", (double) rc.rssi, (double) rc.channels[0], (double) rc.channels[1], 
                                                                                                                  (double) rc.channels[2], (double) rc.channels[3], (double) rc.channels[4]);

    /*if (!lpos.xy_valid || !lpos.z_valid || lpos.z - current_setpoint.z > SHUTOFF_H || hrt_absolute_time() - start_time > E_TIMEOUT ||
        (hrt_absolute_time() - start_time > TIMEOUT && (lpos.z - initialZ) > LAND_THRESHOLD)) {

      vehicle_setpoint_pub.get().yaw_sp_move_rate = 0.0f;
      vehicle_setpoint_pub.get().thrust = 0.0f;
      vehicle_setpoint_pub.get().fw_control_yaw = false;
      vehicle_setpoint_pub.get().disable_mc_yaw_control = false;
      vehicle_setpoint_pub.get().apply_flaps = false;
      vehicle_setpoint_pub.update();

      actuator_armed_pub.get().ready_to_arm = false;
      actuator_armed_pub.get().armed = false;
      actuator_armed_pub.update();
      run_task = false;

      mavlink_and_console_log_info(&mavlink_log_pub, "ABORTING");
      break;

      PX4_LOG("LOST FLOW ESTIMATION, RUNNING ATTITUDE HOLD");

      control_mode_pub.get().flag_control_position_enabled = false;
      control_mode_pub.get().flag_control_velocity_enabled = false;
      control_mode_pub.get().flag_control_acceleration_enabled = false;
      control_mode_pub.get().flag_control_offboard_enabled = true;

      control_mode_pub.get().flag_control_altitude_enabled = false;
      control_mode_pub.get().flag_control_attitude_enabled = true;
      control_mode_pub.get().flag_control_rates_enabled = true;

      control_mode_pub.update();

      //SET VEHICLE SETPOINT
      vehicle_setpoint_pub.get().roll_body = 0.0f;
      vehicle_setpoint_pub.get().pitch_body = 0.0f;
      vehicle_setpoint_pub.get().yaw_body = 0.0f;
      vehicle_setpoint_pub.get().thrust = 0.58f;
      setQuaternionAttSetpoint(vehicle_setpoint_pub.get(), 0.0, 0.0, 0.0);
      vehicle_setpoint_pub.update();

    } else if (hrt_absolute_time() - start_time > TIMEOUT) {
      // LANDING TIME
      current_setpoint.x = lpos.x;
      current_setpoint.y = lpos.y;
      current_setpoint.z = 0.0f;
      current_setpoint.yaw = lpos.yaw;

      pos_setpoint_pub.get().current = current_setpoint;
      pos_setpoint_pub.get().current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
      pos_setpoint_pub.update();
      mavlink_and_console_log_info(&mavlink_log_pub, "LANDING");
    } else {
      // ACTIVATE POSITION CONTROL MODE
      waypoints = GetWaypoints(initialZ, lpos.yaw);
      current_setpoint = waypoints[waypoint_index % waypoints.size()];

      // MAINTAIN CURRENT X Y WHATEVER IT IS
      //current_setpoint.x = lpos.x;
      //current_setpoint.y = lpos.y;

      current_setpoint.disable_mc_yaw_control = true;

      pos_setpoint_pub.get().current = current_setpoint;
      pos_setpoint_pub.get().current.type = position_setpoint_s::SETPOINT_TYPE_OFFBOARD;
      pos_setpoint_pub.update();

      control_mode_pub.get().flag_armed = true;
      control_mode_pub.get().flag_control_offboard_enabled = true;
      control_mode_pub.get().flag_control_position_enabled = true;
      control_mode_pub.get().flag_control_altitude_enabled = true;
      control_mode_pub.get().flag_control_attitude_enabled = true;
      control_mode_pub.get().flag_control_velocity_enabled = true;
      control_mode_pub.get().flag_control_climb_rate_enabled = true;
      control_mode_pub.get().flag_control_rates_enabled = true;
      control_mode_pub.update();
    }

    if (dist < TOLERANCE_RADIUS) {
      ++waypoint_index;
      mavlink_and_console_log_info(&mavlink_log_pub,
            "Switching setpoint to: %f, %f, %f",
            (double) current_setpoint.x,
            (double) current_setpoint.y,
            (double) current_setpoint.z);
    } */

    usleep(50000);
  }
  // vehicle_setpoint_pub.get().yaw_sp_move_rate = 0.0f;
  // vehicle_setpoint_pub.get().thrust = 0.0f;
  // vehicle_setpoint_pub.get().fw_control_yaw = false;
  // vehicle_setpoint_pub.get().disable_mc_yaw_control = false;
  // vehicle_setpoint_pub.get().apply_flaps = false;
  // vehicle_setpoint_pub.update();
  actuator_armed_pub.get().ready_to_arm = false;
  actuator_armed_pub.get().armed = false;
  actuator_armed_pub.update();
  run_task = false;
  return 0;
}