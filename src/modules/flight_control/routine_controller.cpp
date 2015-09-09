#include "routine_controller.h"

#include <stdio.h>

void RoutineController::SetCurrentRoutine(routine_codes::Code routine_code) {
  if(routine_code >= routine_codes::kRoutinesSize) {
    printf("Error: Invalid routine: %d\n", routine_code);
    current_routine_ = routine_list[routine_codes::kDefaultRoutine];
  } else {
    current_routine_ = routine_list[routine_code];
  }
  printf("Info: Switching to routine: %s\n", current_routine_name());
}

void RoutineController::ExecuteCurrentRoutine() {
  if(current_routine_->ExecuteCycle()) {
    if(current_routine_->routine_code() == routine_codes::kLanding) {
      SetCurrentRoutine(routine_codes::kKillMotors);
    } else {
      SetCurrentRoutine(routine_codes::kDefaultRoutine);
    }
  }
}
