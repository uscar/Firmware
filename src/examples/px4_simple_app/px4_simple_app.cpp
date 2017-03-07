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
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/battery_status.h>


extern "C" __EXPORT int px4_simple_app_main(int argc, char *argv[]);

static bool run_task = false;
static int daemon_task;

int main_thread(int argc, char* argv[]);
void setQuaternionAttSetpoint(vehicle_attitude_setpoint_s & set, double roll, double pitch, double yaw);

int px4_simple_app_main(int argc, char *argv[])
{

 if (argc < 2) {
   PX4_INFO("Usage: px4_simple_app <start> | <stop>");
   return 1;
 }

 if (!strcmp(argv[1], "start")) {
   if (!run_task) {
        PX4_INFO("Starting testing application...");
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
   PX4_INFO("Usage: px4_simple_app <start> | <stop>");
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
	PX4_INFO("Starting testing application...");
  uORB::Subscription<optical_flow_s> flow_sub(ORB_ID(optical_flow));
  uORB::Subscription<home_position_s> home_pos_sub(ORB_ID(home_position));
  uORB::Subscription<sensor_combined_s> sensor_sub(ORB_ID(sensor_combined));


  uORB::Subscription<vehicle_attitude_s> vehicle_attitude_sub(ORB_ID(vehicle_attitude));

  uORB::Subscription<vehicle_attitude_setpoint_s> vehicle_setpoint_sub(ORB_ID(vehicle_attitude_setpoint));

  uORB::Subscription<battery_status_s> battery_sub(ORB_ID(battery_status));

  uORB::Publication<vehicle_local_position_setpoint_s> loc_setpoint(ORB_ID(vehicle_local_position_setpoint));

  uORB::Publication<vehicle_attitude_setpoint_s> vehicle_setpoint_pub(
      ORB_ID(vehicle_attitude_setpoint), 0);

  uORB::Publication<position_setpoint_triplet_s> pos_setpoint_pub(
      ORB_ID(position_setpoint_triplet), -1);
  uORB::Publication<vehicle_control_mode_s> control_mode_pub(
      ORB_ID(vehicle_control_mode), -1);

  uORB::Publication<actuator_armed_s> actuator_armed_pub(
      ORB_ID(actuator_armed), -1);

  
  // actuator_armed_pub.get().ready_to_arm = true;
  actuator_armed_pub.get().armed = true;
  actuator_armed_pub.update();

  control_mode_pub.get().flag_control_manual_enabled = false;
  control_mode_pub.get().flag_control_auto_enabled  = false;
  control_mode_pub.get().flag_control_offboard_enabled = false;
  control_mode_pub.get().flag_control_position_enabled = true;
  control_mode_pub.get().flag_control_altitude_enabled = false;
  control_mode_pub.get().flag_control_rates_enabled = true;
  control_mode_pub.get().flag_control_attitude_enabled = true;
  control_mode_pub.get().flag_control_rattitude_enabled = true;
  control_mode_pub.get().flag_control_force_enabled = true;

  control_mode_pub.update();

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

  double thrust = 0.4;
  double pitch_set = 0.0;
  float MAX_HEIGHT = 0.8f;


  vehicle_attitude_sub.update();
  vehicle_setpoint_sub.update();
  sensor_sub.update();
  flow_sub.update();
  battery_sub.update();
  usleep(1000000);


  double w = vehicle_attitude_sub.get().q[0];
  double x = vehicle_attitude_sub.get().q[1];
  double y = vehicle_attitude_sub.get().q[2];
  double z = vehicle_attitude_sub.get().q[3];

  double ysqr = x * x;
  double t3 = +2.0 * (w * z + x * y);
  double t4 = +1.0 - 2.0 * (ysqr + z * z);  
  double initial_yaw = std::atan2(t3, t4);
    
  while (run_task) {
    vehicle_attitude_sub.update();
    vehicle_setpoint_sub.update();
    sensor_sub.update();
    flow_sub.update();
    battery_sub.update();

    w = vehicle_attitude_sub.get().q[0];
    x = vehicle_attitude_sub.get().q[1];
    y = vehicle_attitude_sub.get().q[2];
    z = vehicle_attitude_sub.get().q[3];

    ysqr = x * x;
    // roll (x-axis rotation)
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + ysqr);
    double roll_act = std::atan2(t0, t1);

    // pitch (y-axis rotation)
    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    double pitch_act = std::asin(t2);

    // yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y);
    t4 = +1.0 - 2.0 * (ysqr + z * z);  
    double yaw_act = std::atan2(t3, t4);

    /*
    struct pollfd fds;
    fds.fd = 0; //stdin
    fds.events = POLLIN;

     // abort on user request 
    char c = 0;
    int ret = poll(&fds, 1, 100);
    if (ret > 0)
        read(0, &c, 1);
    if (c == 0x03 || c == 0x63 || c == 'q' ||
    */



    // double roll = vehicle_setpoint_sub.get().roll_body;
    // double pitch = vehicle_setpoint_sub.get().pitch_body;
    // double yaw = vehicle_setpoint_sub.get().yaw_body;
    // double thrust_set = vehicle_setpoint_sub.get().thrust;
    //PX4_INFO("Set Roll: %8.3f\tPitch: %8.3f\tYaw: %8.3f\tThrust: %8.3f\t", roll, pitch, yaw, thrust_set);
    PX4_INFO("A R: %8.3f\tP: %8.3f\tY: %8.3f\tH: %8.3f\tBv: %8.3f\tBa: %8.3f\t", roll_act, pitch_act, yaw_act, 
      (double) flow_sub.get().ground_distance_m, 
      (double) battery_sub.get().voltage_filtered_v, 
      (double)battery_sub.get().current_filtered_a);
    //PX4_INFO("Err Roll: %8.3f\tPitch: %8.3f\tYaw: %8.3f\t", roll_act - roll, pitch_act - pitch, yaw_act - yaw);

    PX4_INFO("Flow quality: %d\t X: %8.3f\t Y: %8.3f\t", flow_sub.get().quality, 
      (double) flow_sub.get().pixel_flow_x_integral, 
      (double) flow_sub.get().pixel_flow_y_integral);

    //SET VEHICLE SETPOINT
    vehicle_setpoint_pub.get().roll_body = 0.0;
    vehicle_setpoint_pub.get().pitch_body = (float) pitch_set;
    vehicle_setpoint_pub.get().yaw_body = (float) initial_yaw;//(float) yaw_act;
    vehicle_setpoint_pub.get().yaw_sp_move_rate = 0.2;
    vehicle_setpoint_pub.get().thrust = (float) thrust;
    setQuaternionAttSetpoint(vehicle_setpoint_pub.get(), 0.0, pitch_set, initial_yaw);
    vehicle_setpoint_pub.update();

    loc_setpoint.get().x = 0.0f;
    loc_setpoint.get().y = 0.0f;
    loc_setpoint.get().z = 0.5f;
    loc_setpoint.update();

    //pitch_set += 0.0005;
    // thrust += 0.001;

    // if (thrust > 0.95)
    //   thrust = 0.95;

    if (flow_sub.get().ground_distance_m > MAX_HEIGHT
         || fabs(roll_act) > 0.5 || fabs(pitch_act) > 0.5 || thrust > 1.5) {
      break;
    }

    usleep(100000);
  }

  //PX4_INFO("Aborting %8.4f     %8.4f", (double) flow.ground_distance_m, 
  //                                         (double) sensors.baro_alt_meter);
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

  return 0;
}
