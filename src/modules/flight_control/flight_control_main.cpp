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

#include "./routines/routine_codes.h"
#include "routine_controller.h"

extern "C" __EXPORT int flight_control_main(int argc, char* argv[]);

int flight_control_main(int argc, char* argv[]) {
  RoutineController routine_controller;
  while (true) {
    // Read and parse serialized data from mavlink

    // Publish routine parameters from input

    routine_controller.ExecuteCurrentRoutine();

    // Re-poll parameters and send data through mavlink
  }
  return 0;
}
