#pragma once

#ifndef __KICIRCLE_H
#define __KICIRCLE_H
 

#include "KIProcedure.h"

class KICircle : public KIProcedure
{
private:
	double radius;
	
	bool isCompleted;

	DronePosition startPosition;
	DronePosition checkPosition;
	TooN::Vector<3> centerPoint;
	TooN::Vector<3> upVector;
	double lineSpeed;
	double targetAngle;

public:
	KICircle(DronePosition startPositionP,
		TooN::Vector<3> centerPointP,
		TooN::Vector<3> upVectorP,
		double lineSpeedP = 0.5,
		double angleP = 360);

	~KICircle(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KICIRCLE_H */
