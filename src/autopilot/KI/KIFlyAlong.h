#pragma once

#ifndef __KIFLYTALONG_H
#define __KIFLYTALONG_H
 

#include "KIProcedure.h"

class KIFlyAlong : public KIProcedure
{
private:
	int startAtClock;
	bool directionSet;
	bool isCompleted;

	int stayTimeMs;
	double maxControlFactor;

	DronePosition checkpoint;
	DronePosition direction;
	double lineSpeed;

public:
	KIFlyAlong(DronePosition checkpointP, 
		DronePosition directionP,
		double lineSpeedP = 0.1,
		double stayTime = 2,
		double maxControlFactorP = 1);

	~KIFlyAlong(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIFLYALONG_H */
