#ifndef TAKEOFF_H_
#define TAKEOFF_H_

#include "routine.h"

class Takeoff : public Routine {
public:
  Takeoff() : Routine(routine_codes::kTakeoff, "TAKEOFF") { }
  ~Takeoff() { }

  bool ExecuteCycle();

  float curr_throttle() { return curr_throttle_; }

private:
  float curr_throttle_;
};

#endif
