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
#include <drivers/drv_hrt.h>
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
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/commander_state.h>
extern "C" __EXPORT int alt_ctl_main(int argc, char *argv[]);
static bool run_task = false;
static int daemon_task;
int alt_main_thread(int argc, char *argv[]);
void set_attitude_quaternion(
    vehicle_attitude_setpoint_s& setpoint, float roll, float pitch, float yaw);
void set_attitude_quaternion(
    vehicle_attitude_setpoint_s& setpoint, float roll, float pitch, float yaw) {
  double t0 = std::cos(yaw * 0.5f);
  double t1 = std::sin(yaw * 0.5f);
  double t2 = std::cos(roll * 0.5f);
  double t3 = std::sin(roll * 0.5f);
  double t4 = std::cos(pitch * 0.5f);
  double t5 = std::sin(pitch * 0.5f);
  setpoint.q_d[0] = t0 * t2 * t4 + t1 * t3 * t5;
  setpoint.q_d[1] = t0 * t3 * t4 - t1 * t2 * t5;
  setpoint.q_d[2] = t0 * t2 * t5 + t1 * t3 * t4;
  setpoint.q_d[3] = t1 * t2 * t4 - t0 * t3 * t5;
  setpoint.q_d_valid = true;
}
int alt_main_thread(int argc, char *argv[]) {
  uORB::Subscription<vehicle_local_position_s> vehicle_local_position_sub(ORB_ID(vehicle_local_position));
  uORB::Subscription<vehicle_attitude_s> vehicle_attitude_sub(ORB_ID(vehicle_attitude));
  uORB::Publication<vehicle_attitude_setpoint_s> vehicle_attitude_setpoint_pub(
      ORB_ID(vehicle_attitude_setpoint), 0);
  uORB::Publication<vehicle_control_mode_s> vehicle_control_mode_pub(
      ORB_ID(vehicle_control_mode), 0);
  usleep(2000000);
  constexpr float ALT_SETPOINT = 0.4f;
  constexpr float P_GAIN = 0.5f;
  constexpr float I_GAIN = 0.00f;
  float i_sum = 0.0f;
  
  constexpr float D_GAIN = 0.00f;
  float d_last_err = 0.0f;
  float thrust = 0.0;
  int logging_counter = 0;
  float last_run = hrt_absolute_time();
  while (run_task) {
    usleep(1000);
    float curr_time = hrt_absolute_time();
    float dt = curr_time - last_run;
    last_run = curr_time;
    
    vehicle_local_position_sub.update();
    auto lpos = vehicle_local_position_sub.get();
    float height = -lpos.z;
    float height_err = sqrt(fabs(ALT_SETPOINT - height));
    if (ALT_SETPOINT - height < 0)
      height_err = -height_err;
    set_attitude_quaternion(vehicle_attitude_setpoint_pub.get(), 0.0f, 0.0f, 0.0f);

    i_sum += height_err;

    thrust = 0.4f + P_GAIN * height_err;
    thrust += I_GAIN * i_sum;
    if (fabs(d_last_err) < 0.000000001) d_last_err = height_err;
    thrust += D_GAIN * (height_err - d_last_err) / dt;

    if (thrust > 0.8f)
      thrust = 0.8f;
    if (thrust < 0.2f)
      thrust = 0.2f;

    d_last_err = height_err;
    vehicle_attitude_setpoint_pub.get().thrust = thrust;
    vehicle_attitude_setpoint_pub.update();
    if ((logging_counter++ << 22) == 0) {
      if (!lpos.z_valid)
        PX4_INFO("Z position is not valid");
      PX4_INFO("lpos (x, y, z, thrust): %8.4f, %8.4f, %8.4f, %8.4f", (double) lpos.x, (double) lpos.y, (double) height, (double) thrust);
    }
  }
  
  run_task = false;
  return 0;
}
int alt_ctl_main(int argc, char *argv[]) {
  if (argc < 2) {
    PX4_INFO("Usage: alt_ctl <start> | <stop>");
    return 1;
  }
  if (!strcmp(argv[1], "start")) {
    if (!run_task) {
      PX4_INFO("Starting testing application...");
      run_task = true;
      daemon_task = px4_task_spawn_cmd("alt_ctl",
                 SCHED_DEFAULT,
                 SCHED_PRIORITY_MAX - 5,
                 13500,
                 alt_main_thread,
                 (argv && argc > 2) ? 
                 (char *const *) &argv[2] : (char *const *) nullptr);
    }
  } else if (!strcmp(argv[1], "stop")) {
    run_task = false;
  } else {
    PX4_INFO("Usage: alt_ctl <start> | <stop>");
  }
  return 0;
}