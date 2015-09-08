#ifndef FLIGHT_CONTROL_ROUTINES_ROUTINE_CODES_H_
#define FLIGHT_CONTROL_ROUTINES_ROUTINE_CODES_H_

// Define the available routines.

namespace routine_codes {

typedef enum {
  kTakeoff,
  kAltitudeHold,
  kLanding,
  kKillMotors,
  kDefaultRoutine,
  kRoutinesSize // Always keep this as the last element in the enum,
                // it acts as the number of available routines.
} Code;

};

#endif
