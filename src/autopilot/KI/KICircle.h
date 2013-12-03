#pragma once

#ifndef __KICIRCLE_H
#define __KICIRCLE_H
 

#include "KIProcedure.h"

class KICircle : public KIProcedure
{
private:
	double radius;
	
	int stayTimeMs;
	int startedAtClock;
	bool isCompleted;

	DronePosition checkPosition;
	TooN::Vector<3> centerPoint;
	TooN::Vector<3> upVector;
	double lineSpeed;

public:
	KICircle(DronePosition startPositionP,
		TooN::Vector<3> centerPointP,
		TooN::Vector<3> upVectorP,
		double lineSpeedP = 0.5,
		double stayTimeP = 20);

	~KICircle(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KICIRCLE_H */
