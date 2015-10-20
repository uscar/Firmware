/*
 * USCARPositionPublisher.h
 *
 *  Created on: Sep 30, 2015
 *      Author: Vadim
 */

#ifndef USCARPOSITIONPUBLISHER_H_
#define USCARPOSITIONPUBLISHER_H_

#include <uORB/topics/vehicle_gps_position.h>

class USCARPositionPublisher {
public:
	virtual ~USCARPositionPublisher();

	orb_advert_t    _custom_pos_pub;
	struct vehicle_gps_position_s    _custom_gps_msg;
	void publishUSCARPosData();
	static USCARPositionPublisher* Get();
	static void main_trampoline(int argc, char *argv[]);
	void publishStart();
protected:
	static USCARPositionPublisher* publisher_pointer;
private:
	USCARPositionPublisher();
	long long counter;
};

#endif /* USCARPOSITIONPUBLISHER_H_ */
