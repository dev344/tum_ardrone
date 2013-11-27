#pragma once

#ifndef __KIFLYTALONG_H
#define __KIFLYTALONG_H
 

#include "KIProcedure.h"

class KIFlyAlong : public KIProcedure
{
private:
	bool directionSet;
	bool isCompleted;

	DronePosition startPosition;
	TooN::Vector<3> checkpoint;
	TooN::Vector<3> direction;
	double lineSpeed;
	double distance;

public:
	KIFlyAlong(DronePosition startPositionP,
		TooN::Vector<3> directionP,
		double lineSpeedP = 0.5,
		double distanceP = 1);

	~KIFlyAlong(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIFLYALONG_H */
