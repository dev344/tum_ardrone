#pragma once

#ifndef __KICIRCLE_H
#define __KICIRCLE_H
 

#include "KIProcedure.h"

class KICircle : public KIProcedure
{
private:
	double radius;
	
	int startAtClock;
	bool isCompleted;

	int stayTimeMs;
	double maxControlFactor;

	DronePosition checkpoint;
	TooN::Vector<3> centerpos;
	TooN::Vector<3> upVector;
	double lineSpeed;

public:
	KICircle(DronePosition checkpointP, 
		TooN::Vector<3> centerposP,
		TooN::Vector<3> upVectorP,
		double lineSpeedP = 0.1,
		double stayTime = 2,
		double maxControlFactorP = 1);

	~KICircle(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KICIRCLE_H */
