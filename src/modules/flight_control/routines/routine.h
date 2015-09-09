#ifndef ROUTINES_ROUTINE_H_
#define ROUTINES_ROUTINE_H_

#include <string>

#include "routine_codes.h"

class Routine {
public:
  Routine(const routine_codes::Code _routine_code, const char* _name)
      : routine_code_(_routine_code), name_(_name) { }

  virtual ~Routine() { }

  // Returns true if finished with execution
  // (i.e should set to default routine), false otherwise
  virtual bool ExecuteCycle() = 0;

  // Executes cycles until it returns true. Not recommended b/c uninterruptable,
  // but could be useful if something NEEDS to be done (i.e. emergency landing).
  bool ExecuteFull() {
    while(!ExecuteCycle());
    return true;
  }

  // Each routine should update parameters after every cycle from whatever
  // uORB subscriptions is might have.
  virtual void UpdateParameters() { }

  const char* name() const { return name_; }
  routine_codes::Code routine_code() const { return routine_code_; }

protected:
  const routine_codes::Code routine_code_;
  const char* name_;

  int count_; // Used for periodic debugging.
  
  static constexpr bool DEBUG = true; // Enable debugging function call
};

#endif
