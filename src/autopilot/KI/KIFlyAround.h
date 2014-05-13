#pragma once

#ifndef __KIFLYAROUND_H
#define __KIFLYAROUND_H

#include "KIProcedure.h"

class KIFlyAround: public KIProcedure {
private:
	TooN::Vector<3> mv3CenterPoint;
	double mdRadius;
	double mdStartYawAngle;
	double mdTotalYawAngle;
	TooN::Vector<3> mv3CirclePlaneNormVector;
	double mdLinearSpeed;

	double mdYawAngleCheckPoint;
	double mdApproxDistance;

	bool isCompleted;

	ControlCommand ctrlCmdAlongCircle(TooN::Vector<3> directionUnitVector,
			double targetYaw, double currentYaw);
	bool isTotalYawAngleAchieved(double currentYaw);

public:
	KIFlyAround(TooN::Vector<3> centerPoint, double radius,
			double startYawAngle, double totalYawAngle,
			TooN::Vector<3> circlePlaneNormVector, double linearSpeed);

	~KIFlyAround(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIFLYAROUND_H */
