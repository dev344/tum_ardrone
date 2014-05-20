/*
 * KIZigZagTower.h
 *
 *  Created on: May 13, 2014
 *      Author: ziquan
 */

#pragma once
#ifndef __KIZIGZAGTOWER_H_
#define __KIZIGZAGTOWER_H_

#include "KIProcedure.h"
#include "KIFlyAlong.h"
#include "KIFlyAround.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

class KIZigZagTower: public KIProcedure {
private:
	TooN::Vector<3> mv3TopPoint, mv3BtmPoint;
	double mdAngleStart, mdAngleTotal;
	int miNumOfRows, miNumOfCols;
	double mdAngleH, mdAngleV;

	double mdRowHeight, mdDistToTower;
	double mdMinDistToTower;

	//vector<TooN::Vector<3> > mvv3LeftToRightUnitVectors;
	//vector<TooN::Vector<3> > mvv3OutwardNormVectors;
	vector<TooN::Vector<3> > mvv3LandMarks;
	vector<double> mvdYawAngleWayPoints;

	int miCurrentWayPointNum;

	bool isCompleted;

	int miTimer;	// compute time used between two snapshot

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
	KIProcedure* mpKIHelper;

public:
	KIZigZagTower(TooN::Vector<3> topPoint, TooN::Vector<3> btmPoint,
			double angleStart, double angleTotal, int numOfRows,
			double angleH, double angleV);

	~KIZigZagTower(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIZIGZAGTOWER_H_ */
