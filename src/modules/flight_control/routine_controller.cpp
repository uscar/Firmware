#include "routine_controller.h"

#include <stdio.h>

void RoutineController::SetCurrentRoutine(routine_codes::Code routine_code) {
  if(routine_code >= routine_codes::kRoutinesSize) {
    curr_routine_ = routine_list[routine_codes::kDefaultRoutine];
  } else {
    curr_routine_ = routine_list[routine_code];
  }
  printf("Switching to routine: %s\n", curr_routine_name().c_str());
}

void RoutineController::ExecuteCurrRoutine() {
  if(curr_routine_->ExecuteCycle()) {
    if(curr_routine_->routine_code() == routine_codes::kLanding) {
      SetCurrentRoutine(routine_codes::kKillMotors);
    } else {
      SetCurrentRoutine(routine_codes::kDefaultRoutine);
    }
  }
}
