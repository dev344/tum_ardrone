#pragma once

#ifndef __KIFLYTALONG_H
#define __KIFLYTALONG_H

#include "KIProcedure.h"

class KIFlyAlong: public KIProcedure {
private:

	DronePosition mposeStart;
	DronePosition mposeEnd;
	double mdLinearSpeed;

	TooN::Vector<3> mv3DirectionUnitVector;
	double mdDistance;

    bool isNear;
	bool isCompleted;

	ControlCommand ctrlCmdAlongDirection(vector<3> error, double yaw);

public:
	KIFlyAlong(DronePosition startPose, DronePosition endPose,
			double linearSpeed);

	~KIFlyAlong(void);

	bool update(const tum_ardrone::filter_stateConstPtr statePtr);

	inline double getDistance() { return mdDistance; }
};

#endif /* __KIFLYALONG_H */
