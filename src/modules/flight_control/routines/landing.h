#ifndef LANDING_H_
#define LANDING_H_

#include "routine.h"

class Landing : public Routine {
public:
	Landing() : Routine(routine_codes::kLanding, "LANDING") { }
	~Landing() { }

	bool ExecuteCycle();

private:
	static constexpr float END_HEIGHT = 0.05;
};

#endif
