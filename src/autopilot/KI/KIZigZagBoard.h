#pragma once
#ifndef __KIZIGZAGBOARD_H
#define __KIZIGZAGBOARD_H

#include "KIProcedure.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

class KIZigZagBoard: public KIProcedure {
private:
	TooN::Vector<3> mv3TopLPoint, mv3TopRPoint, mv3BtmLPoint, mv3BtmRPoint;
	int	miNumOfRows, miNumOfCols;
	double mdAngleH, mdAngleV;

	TooN::Vector<3> mv3OutwardNormVector;
	double mdYawAngle;

	vector<vector<TooN::Vector<3> > > mvvv3WayPoints;
	queue<DronePosition> qPoses;

	int reachedAtClock;
	bool reached;
	bool isStarted;
	bool isCompleted;

	int stayTimeMs;
	double initialReachedDist;
	double stayWithinDist;

	DronePosition checkpoint;

public:
	KIZigZagBoard(TooN::Vector<3> topLPoint,
			TooN::Vector<3> topRPoint,
			TooN::Vector<3> btmLPoint,
			TooN::Vector<3> btmRPoint,
			int numOfRows,
			double angleH,
			double angleV);

	~KIZigZagBoard(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIZIGZAGBOARD_H */
