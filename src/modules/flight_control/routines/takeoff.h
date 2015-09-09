#ifndef TAKEOFF_H_
#define TAKEOFF_H_

#include "routine.h"

class Takeoff : public Routine {
public:
  Takeoff() : Routine(routine_codes::kTakeoff, "TAKEOFF") { }
  ~Takeoff() { }

  bool ExecuteCycle();

  float current_throttle() { return current_throttle_; }

private:
  float current_throttle_;
};

#endif
