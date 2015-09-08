#ifndef KILL_MOTORS_H_
#define KILL_MOTORS_H_

#include "routine.h"

class KillMotors : public Routine {
public:
  KillMotors() : Routine(routine_codes::kKillMotors, "KILL MOTORS") { }
  ~KillMotors() { }

  bool ExecuteCycle() {
    return false;
  }
  
private:

};
#endif
