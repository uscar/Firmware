#ifndef FLIGHT_CONTROL_ROUTINE_CONTROLLER_H_
#define FLIGHT_CONTROL_ROUTINE_CONTROLLER_H_

#include <string>

#include "./routines/routine_codes.h"
#include "./routines/altitude_hold.h"
#include "./routines/kill_motors.h"
#include "./routines/landing.h"
#include "./routines/routine.h"
#include "./routines/takeoff.h"

class RoutineController {
public:
  RoutineController() {
    routine_list = new Routine*[routine_codes::kRoutinesSize];
    routine_list[routine_codes::kTakeoff] = new Takeoff();
    routine_list[routine_codes::kAltitudeHold] = new AltitudeHold();
    routine_list[routine_codes::kLanding] = new Landing();
    routine_list[routine_codes::kKillMotors] = new KillMotors();
    routine_list[routine_codes::kDefaultRoutine]
        = routine_list[routine_codes::kAltitudeHold];
    SetCurrentRoutine(routine_codes::kKillMotors);
  }

  ~RoutineController() {
    for(int i = 0; i < routine_codes::kRoutinesSize; ++i) {
      delete routine_list[i];
    }
    delete [] routine_list;
  }

  void ExecuteCurrentRoutine();

  void SetCurrentRoutine(routine_codes::Code code);

  const char* current_routine_name() const {
    return current_routine_->name();
  }
private:
  Routine* current_routine_;
  Routine** routine_list;
};

#endif
