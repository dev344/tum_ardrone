/*
 * KIZigZagTower.cpp
 *
 *  Created on: May 16, 2014
 *      Author: ziquan
 */

#include "KIZigZagTower.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"
#include "std_msgs/String.h"
#include <sstream>

bool KIZigZagTower::isWayPointReached(int waypointNum, DronePosition pose) {
	int rowNum = waypointNumToRowNum(waypointNum);
	int colNum = waypointNumToColNum(waypointNum);

	double yawAngleCenter = vectorToYaw(mvv3LandMarks[rowNum] - pose.pos)
			- pose.yaw;
	double pitchAngleTop = vectorToPitch(mvv3LandMarks[rowNum] - pose.pos);
	double pitchAngleBtm = vectorToPitch(
			mvv3LandMarks[rowNum] + mdRowHeight * TooN::makeVector(0, 0, -1)
					- pose.pos);

	// check yaw diff
	if (fabs(pose.yaw - mvdYawAngleWayPoints[colNum]) > 10) {
		return false;
	}
	if (yawAngleCenter < -mdAngleH / 4 || yawAngleCenter > mdAngleH / 4) {
		return false;
	}
	if (pitchAngleTop < -mdAngleV / 2 || pitchAngleTop > -mdAngleV / 5) {
		return false;
	}
	if (pitchAngleBtm > mdAngleV / 2 || pitchAngleBtm < mdAngleV / 5) {
		return false;
	}
	if (pitchAngleBtm - pitchAngleTop < 0.5 * mdAngleV
			|| pitchAngleBtm - pitchAngleTop > 0.7 * mdAngleV) {
		//return false;
	}

	cout << "YawDiff: " << fabs(pose.yaw - mvdYawAngleWayPoints[colNum])
			<< endl;
	cout << "Center: " << yawAngleCenter << endl;
	cout << "T: " << pitchAngleTop << "\tB: " << pitchAngleBtm << "\tB-T: "
			<< (pitchAngleBtm - pitchAngleTop) << endl;
	return true;
}

KIZigZagTower::KIZigZagTower(TooN::Vector<3> topPoint, TooN::Vector<3> btmPoint,
		double angleStart, double angleTotal, int numOfRows, double angleH,
		double angleV) {

	mv3TopPoint = topPoint;
	mv3BtmPoint = btmPoint;
	mdAngleStart = angleStart;
	mdAngleTotal = angleTotal;

	miNumOfRows = numOfRows;
	mdAngleH = angleH;
	mdAngleV = angleV;

	cout << "TOWER TOP, BOTTOM, AngleStart, AngleTotal" << endl << mv3TopPoint
			<< "\t" << mv3BtmPoint << endl << mdAngleStart << "\t"
			<< mdAngleTotal << endl;
	cout << "ROWS\t" << miNumOfRows << endl;
	cout << "H:V\t" << mdAngleH << ":" << mdAngleV << endl;

	// landmarks
	TooN::Vector<3> rowHeightVector = (mv3BtmPoint - mv3TopPoint) / miNumOfRows;
	mvv3LandMarks.push_back(mv3TopPoint);
	for (int i = 1; i < miNumOfRows; i++) {
		mvv3LandMarks.push_back(mv3TopPoint + i * rowHeightVector);
	}

	// row height for each camera view
	mdMinDistToTower = 1;
	double tanHalfAngleV = sin(mdAngleV / 2 * M_PI / 180)
			/ cos(mdAngleV / 2 * M_PI / 180);
	double minRowHeight = mdMinDistToTower * 2 * tanHalfAngleV * 0.6;
	mdRowHeight = max(minRowHeight, -rowHeightVector[2]);
	mdDistToTower = mdRowHeight / 2 / tanHalfAngleV / 0.6;
	double tanHalfAngleH = sin(mdAngleH / 2 * M_PI / 180)
			/ cos(mdAngleH / 2 * M_PI / 180);
	//mdColWidth = mdDistToBoard * 2 * tanHalfAngleH * 0.6;
	cout << "RowHeight:DistToBoard\t" << mdRowHeight << ":" << mdDistToTower
			<< endl;

	// number of cols
	miNumOfCols = (int) ceil(mdAngleTotal / 45) + 1;
	cout << "Num of rows:cols\t" << miNumOfRows << ":" << miNumOfCols << endl;

	// yaw angle waypoints
	mvdYawAngleWayPoints.push_back(mdAngleStart);
	for (int i = 1; i < miNumOfCols; i++) {
		mvdYawAngleWayPoints.push_back(
				mdAngleStart - i * mdAngleTotal / (miNumOfCols - 1));
	}

	cout << "Yaw angle waypoints:";
	for (int i = 0; i < miNumOfCols; i++) {
		cout << "\t" << mvdYawAngleWayPoints[i];
	}
	cout << endl;

	cout << "LandMarks:" << endl;
	for (int i = 0; i < miNumOfRows; i++) {
		cout << mvv3LandMarks[i] << endl;
	}

	miCurrentWayPointNum = -1;

	isCompleted = false;
	mpKIHelper = NULL;

	char buf[200];
	snprintf(buf, 200, "zigzagtower");
	command = buf;
}

KIZigZagTower::~KIZigZagTower(void) {
}

bool KIZigZagTower::update(const tum_ardrone::filter_stateConstPtr statePtr) {
	if (isCompleted) {
		mpKIHelper = NULL;
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}

	if (miCurrentWayPointNum < 0) {
		miCurrentWayPointNum = 0;
		miTimer = getMS();

		mpKIHelper = new KIFlyAlong(
				DronePosition(
						TooN::makeVector(statePtr->x, statePtr->y, statePtr->z),
						statePtr->yaw),
				DronePosition(
						mvv3LandMarks[0]
								+ mdRowHeight / 2 * TooN::makeVector(0, 0, -1)
								+ mdDistToTower
										* TooN::makeVector(
												- sin(mdAngleStart * M_PI / 180),
												- cos(mdAngleStart * M_PI / 180),
												0), angleToValidYaw(mdAngleStart)),
				max(0.2 , min(1.0, mdDistToTower / 8))); // safe speed
		mpKIHelper->setPointers(this->node, this->controller);
	}

	DronePosition currentPose = DronePosition(
			TooN::makeVector(statePtr->x, statePtr->y, statePtr->z),
			statePtr->yaw);

	// is reached?
	if (mpKIHelper->update(statePtr) && isWayPointReached(miCurrentWayPointNum, currentPose)) {
		// snap shot
		std_msgs::String s;
		s.data = "Snap ";
		std::stringstream buf;
		int rowNum = waypointNumToRowNum(miCurrentWayPointNum);
		int colNum = waypointNumToColNum(miCurrentWayPointNum);
		buf << rowNum << " " << colNum << " " << statePtr->x << " "
				<< statePtr->y << " " << statePtr->z << " " << statePtr->yaw;
		s.data += buf.str();
		node->interface_directions_pub.publish(s);
		cout << "Waypoint " << miCurrentWayPointNum << " reached after "
				<< (getMS() - miTimer) / 1000 << "s\t" << buf.str() << endl;
		miTimer = getMS();

		// update to the next pose
		miCurrentWayPointNum++;
		if (miCurrentWayPointNum >= miNumOfRows * miNumOfCols) {
			cout << "Zigzagtower complete!" << endl;
			isCompleted = true;
		} else {
			int oldRowNum = waypointNumToRowNum(miCurrentWayPointNum - 1);
			int oldColNum = waypointNumToColNum(miCurrentWayPointNum - 1);

			int newRowNum = waypointNumToRowNum(miCurrentWayPointNum);
			int newColNum = waypointNumToColNum(miCurrentWayPointNum);

			if (oldRowNum == newRowNum) {
				mpKIHelper = new KIFlyAround(
						mvv3LandMarks[newRowNum]
								+ mdRowHeight / 2 * TooN::makeVector(0, 0, -1),
						mdDistToTower, mvdYawAngleWayPoints[oldColNum],
						mdAngleTotal / (miNumOfCols - 1),
						(newRowNum % 2 == 0 ?
								TooN::makeVector(0, 0, 1) :
								TooN::makeVector(0, 0, -1)),
						max(0.2, mdMinDistToTower / 4)); // speed to overcome drift
				mpKIHelper->setPointers(this->node, this->controller);
			} else {
				mpKIHelper = new KIFlyAlong(
						DronePosition(
								TooN::makeVector(statePtr->x, statePtr->y,
										mvv3LandMarks[oldRowNum][2]
												- mdRowHeight / 2),
								mvdYawAngleWayPoints[oldColNum]),
						DronePosition(
								TooN::makeVector(statePtr->x, statePtr->y,
										mvv3LandMarks[newRowNum][2]
												- mdRowHeight / 2),
								mvdYawAngleWayPoints[newColNum]),
						max(0.2, mdMinDistToTower / 4)); // speed to overcome drift
				mpKIHelper->setPointers(this->node, this->controller);
			}
		}
	}

	mpKIHelper->update(statePtr);
	return false;

	// control!
//	node->sendControlToDrone(controller->update(statePtr));
//	return false;	// not done yet (!)
}

