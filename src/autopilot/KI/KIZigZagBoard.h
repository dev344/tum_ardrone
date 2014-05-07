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
	int miNumOfRows, miNumOfCols;
	double mdAngleH, mdAngleV;

	//TooN::Vector<3> mv3OutwardNormVector;
	//double mdYawAngle;
	double mdRowHeight, mdColWidth, mdDistToBoard;

	vector<TooN::Vector<3> > mvv3LeftToRightUnitVectors;
	vector<TooN::Vector<3> > mvv3OutwardNormVectors;
	vector<double> mvdYawAngles;
	vector<vector<TooN::Vector<3> > > mvvv3LandMarks;
	vector<vector<TooN::Vector<3> > > mvvv3WayPoints;
	//vector<DronePosition> mvPoses;
	int miCurrentWayPointNum;

	//int reachedAtClock;
	//bool reached;
	bool isCompleted;

	//int stayTimeMs;
	//double reachedDist;
	//double stayWithinDist;

	DronePosition checkpoint;

	// (0,1,z) -> yaw = 0
	// clockwise -> yaw > 0
	// otherwise -> yaw < 0
	double vectorToYaw(TooN::Vector<3> v);

	// horizon -> pitch = 0
	// downward -> pitch > 0
	// otherwise -> pitch < 0
	double vectorToPitch(TooN::Vector<3> v);

	inline int waypointNumToRowNum(int waypointNum) {
		return waypointNum / miNumOfCols;
	}

	inline int waypointNumToColNum(int waypointNum) {
		if (waypointNumToRowNum(waypointNum) % 2 == 0) {
			return waypointNum % miNumOfCols;
		} else {
			return miNumOfCols - 1 - waypointNum % miNumOfCols;
		}
	}

	bool isWayPointReached(int waypointNum, DronePosition pose);

public:
	KIZigZagBoard(TooN::Vector<3> topLPoint, TooN::Vector<3> topRPoint,
			TooN::Vector<3> btmLPoint, TooN::Vector<3> btmRPoint, int numOfRows,
			double angleH, double angleV);

	~KIZigZagBoard(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIZIGZAGBOARD_H */
