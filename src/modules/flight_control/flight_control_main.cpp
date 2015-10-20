#include <px4_config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <float.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>
#include <math.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/filtered_bottom_flow.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <poll.h>
#include <platforms/px4_defines.h>
#include <time.h>

#include "./routines/routine_codes.h"
#include "routine_controller.h"
#include "USCARPositionPublisher.h"

extern "C" __EXPORT int flight_control_main(int argc, char* argv[]);

USCARPositionPublisher* USCARPositionPublisher::publisher_pointer = NULL;


USCARPositionPublisher::USCARPositionPublisher():
	_custom_pos_pub(nullptr),
	_custom_gps_msg({})
{
	memset(&_custom_gps_msg, 0, sizeof(_custom_gps_msg));
	counter = 0;
	// TODO Auto-generated constructor stub
}

USCARPositionPublisher* USCARPositionPublisher::Get()
{
	if (!publisher_pointer)
	{
		publisher_pointer = new USCARPositionPublisher;
	}
	return publisher_pointer;
}

USCARPositionPublisher::~USCARPositionPublisher() {
	// TODO Auto-generated destructor stub
}

void USCARPositionPublisher::publishUSCARPosData()
{
	//update struct
	_custom_gps_msg.lat = (int32_t) (18e7 - counter * 1e5);
	_custom_gps_msg.lon = (int32_t) (18e7 + counter * 1e5);
	_custom_gps_msg.alt = (int32_t) (18e3 + (double) counter / 1e5);
	_custom_gps_msg.vel_m_s = 7;
	_custom_gps_msg.vel_n_m_s = 7;
	_custom_gps_msg.vel_e_m_s = 7;
	_custom_gps_msg.vel_d_m_s = 7;
	_custom_gps_msg.satellites_used = (uint8_t) 20;

	_custom_gps_msg.timestamp_position = (uint64_t) hrt_absolute_time();
	_custom_gps_msg.timestamp_variance = (uint64_t) hrt_absolute_time();
	_custom_gps_msg.timestamp_velocity = (uint64_t) hrt_absolute_time();
	_custom_gps_msg.timestamp_time = (uint64_t) hrt_absolute_time();
	_custom_gps_msg.time_utc_usec = (uint64_t) hrt_absolute_time();

	/* lazily publish the local position only once available */
	if (_custom_pos_pub != nullptr) {
		/* publish the attitude setpoint */
		orb_publish(ORB_ID(vehicle_gps_position), _custom_pos_pub, &_custom_gps_msg);

	} else {
		/* advertise and publish */
		_custom_pos_pub = orb_advertise(ORB_ID(vehicle_gps_position), &_custom_gps_msg);
	}
	counter++;
}

void USCARPositionPublisher::main_trampoline(int argc, char *argv[])
{
	while (true)
	{
		USCARPositionPublisher::Get()->publishUSCARPosData();
		usleep(500); //was 10
	}
}

void USCARPositionPublisher::publishStart()
{
	px4_task_spawn_cmd("uscar_gps_pub",
					       SCHED_DEFAULT,
					       SCHED_PRIORITY_MAX - 5,
					       1500,
					       (px4_main_t)&USCARPositionPublisher::main_trampoline,
					       nullptr);
}

int flight_control_main(int argc, char* argv[]) {
  //RoutineController routine_controller;
  //intialize pointer to Publisher object that maintrampoline will use to call PosData() func
  printf("START");
  PX4_INFO("running");
  USCARPositionPublisher::Get()->publishStart();
  printf("DONE START");
  //while (true) {

	  //uscarPos.publishUSCARPosData();

    // Read and parse serialized data from mavlink

    // Publish routine parameters from input

    //routine_controller.ExecuteCurrentRoutine();

    // Re-poll parameters and send data through mavlink
  //}
  return 0;
}
