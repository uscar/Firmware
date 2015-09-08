#ifndef ALTITUDE_HOLD_H_
#define ALTITUDE_HOLD_H_

#include "routine.h"

class AltitudeHold : public Routine {
public:
  AltitudeHold() : Routine(routine_codes::kAltitudeHold, "ALTITUDE HOLD") { }
  ~AltitudeHold() { }
  bool ExecuteCycle();

	float hover_throttle() const { return hover_throttle_; }

  float target_height() const { return target_height_; }

private:
  static constexpr float THROTTLE_CORRECTION_SCALE = 0.05;
  static constexpr float ALT_HOLD_H_P = 8.00;
  static constexpr float ALT_HOLD_H_I = 0.00;
  static constexpr float ALT_HOLD_H_D = 8.00;
  static constexpr float ALT_HOLD_H_IMAX = 5;

	float hover_throttle_;
  float target_height_;
};

#endif
