/*
 * KIZigZagBoard.cpp
 *
 *  Created on: Apr 16, 2014
 *      Author: ziquan
 */

#include "KIZigZagBoard.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"
#include "std_msgs/String.h"
#include <sstream>

//double KIZigZagBoard::vectorToYaw(TooN::Vector<3> v) {
//	// base vector = (0,1,0)
//	// yaw angle = arccos ( v<2> . (0,1) / |v<2>| * |(0,1)| )
//	double result = 180.0 / M_PI
//			* acos(v[1] / sqrt(v.slice<0, 2>() * v.slice<0, 2>()));
//
//	// check x value in v to determine sign
//	if (v[0] < 0) {
//		result *= -1;
//	}
//
//	return result;
//}
//
//double KIZigZagBoard::vectorToPitch(TooN::Vector<3> v) {
//	// base vector = (v[0], v[1],0)
//	// pitch angle = arccos ( v . (v[0],v[1],0) / |v| * |(v[0],v[1],0)|
//	double result = 180.0 / M_PI
//			* acos(
//					v.slice<0, 2>() * v.slice<0, 2>()
//							/ (sqrt(v * v)
//									* sqrt(v.slice<0, 2>() * v.slice<0, 2>())));
//	if (v[2] > 0) {
//		result *= -1;
//	}
//	return result;
//}

bool KIZigZagBoard::isWayPointReached(int waypointNum, DronePosition pose) {
	int rowNum = waypointNumToRowNum(waypointNum);
	int colNum = waypointNumToColNum(waypointNum);

	double yawAngleL = vectorToYaw(mvvv3LandMarks[rowNum][colNum] - pose.pos)
			- pose.yaw;

	double yawAngleR = vectorToYaw(
			mvvv3LandMarks[rowNum][colNum]
					+ mdColWidth * mvv3LeftToRightUnitVectors[rowNum]
					- pose.pos) - pose.yaw;
	double pitchAngleTop = vectorToPitch(
			mvvv3LandMarks[rowNum][colNum]
					+ mdColWidth / 2 * mvv3LeftToRightUnitVectors[rowNum]
					- pose.pos);
	double pitchAngleBtm = vectorToPitch(
			mvvv3LandMarks[rowNum][colNum]
					+ mdColWidth / 2 * mvv3LeftToRightUnitVectors[rowNum]
					+ mdRowHeight * TooN::makeVector(0, 0, -1) - pose.pos);

	// check yaw diff
	if (fabs(pose.yaw - mvdYawAngles[rowNum]) > 2) {
		return false;
	}
	if (yawAngleL < -mdAngleH / 2 || yawAngleL > -mdAngleH / 4) {
		return false;
	}
	if (yawAngleR > mdAngleH / 2 || yawAngleR < mdAngleH / 4) {
		return false;
	}
	if (yawAngleR - yawAngleL < 0.5 * mdAngleH
			|| yawAngleR - yawAngleL > 0.7 * mdAngleH) {
//		return false;
	}
	if (pitchAngleTop < -mdAngleV / 2 || pitchAngleTop > -mdAngleV / 4) {
		return false;
	}
	if (pitchAngleBtm > mdAngleV / 2 || pitchAngleBtm < mdAngleV / 4) {
		return false;
	}
	if (pitchAngleBtm - pitchAngleTop < 0.5 * mdAngleV
			|| pitchAngleBtm - pitchAngleTop > 0.7 * mdAngleV) {
//		return false;
	}

	cout << "YawDiff: " << fabs(pose.yaw - mvdYawAngles[rowNum])
			<< "\tDistDiff:"
			<< sqrt(
					(mvvv3WayPoints[rowNum][colNum] - pose.pos)
							* (mvvv3WayPoints[rowNum][colNum] - pose.pos))
			<< endl;
	cout << "L: " << yawAngleL << "\tR: " << yawAngleR << "\tR-L: "
			<< (yawAngleR - yawAngleL) << endl;
	cout << "T: " << pitchAngleTop << "\tB: " << pitchAngleBtm << "\tB-T: "
			<< (pitchAngleBtm - pitchAngleTop) << endl;
	return true;
}

KIZigZagBoard::KIZigZagBoard(TooN::Vector<3> topLPoint,
		TooN::Vector<3> topRPoint, TooN::Vector<3> btmLPoint,
		TooN::Vector<3> btmRPoint, int numOfRows, double angleH,
		double angleV) {

	mv3TopLPoint = topLPoint;
	mv3TopRPoint = topRPoint;
	mv3BtmLPoint = btmLPoint;
	mv3BtmRPoint = btmRPoint;

	miNumOfRows = numOfRows;
	mdAngleH = angleH;
	mdAngleV = angleV;

	cout << "BOARD CORNERS" << endl << mv3TopLPoint << "\t" << mv3TopRPoint
			<< endl << mv3BtmLPoint << "\t" << mv3BtmRPoint << endl;
	cout << "ROWS\t" << miNumOfRows << endl;
	cout << "H:V\t" << mdAngleH << ":" << mdAngleV << endl;

	// board plane outward norm vector
	TooN::Vector<3> tEdgeVector = mv3TopLPoint - mv3TopRPoint;
	TooN::Vector<3> lEdgeVector = mv3BtmLPoint - mv3TopLPoint;
	TooN::Vector<3> bEdgeVector = mv3BtmRPoint - mv3BtmLPoint;
	TooN::Vector<3> rEdgeVector = mv3TopRPoint - mv3BtmRPoint;

	TooN::Vector<3> tlNormVector = TooN::unit(tEdgeVector ^ lEdgeVector);
	TooN::Vector<3> blNormVector = TooN::unit(lEdgeVector ^ bEdgeVector);
	TooN::Vector<3> brNormVector = TooN::unit(bEdgeVector ^ rEdgeVector);
	TooN::Vector<3> trNormVector = TooN::unit(rEdgeVector ^ tEdgeVector);

//	mv3OutwardNormVector = TooN::unit(
//			(tlNormVector + blNormVector + brNormVector + trNormVector) / 4.0);
//
//	cout << "OutwardNormVector\t" << mv3OutwardNormVector << endl;
//
//	mdYawAngle = vectorToYaw(-mv3OutwardNormVector);

//	// yaw angle direction = arccos ( -ourwardNorm_xy . (0,1) / |-ourwardNorm_xy| * |(0,1)| )
//	mdYawAngle =
//			180.0 / M_PI
//					* acos(
//							-mv3OutwardNormVector[1]
//									/ sqrt(
//											mv3OutwardNormVector.slice<0, 2>()
//													* mv3OutwardNormVector.slice<
//															0, 2>()));
//
//    // check x value in diff to determine sign
//    if (mv3OutwardNormVector[0] > 0) {
//        mdYawAngle *= -1;
//    }

//	cout << "yaw\t" << mdYawAngle << endl;

	// leftBoundary and rightBoundary
	vector<TooN::Vector<3> > leftBoundary, rightBoundary;
	TooN::Vector<3> leftLevelHeightVector = (mv3BtmLPoint - mv3TopLPoint)
			/ miNumOfRows;
	TooN::Vector<3> rightLevelHeightVector = (mv3BtmRPoint - mv3TopRPoint)
			/ miNumOfRows;

	leftBoundary.push_back(mv3TopLPoint);
	for (int i = 0; i < miNumOfRows - 1; i++) {
		leftBoundary.push_back(leftBoundary.back() + leftLevelHeightVector);
	}

	rightBoundary.push_back(mv3TopRPoint);
	for (int i = 0; i < miNumOfRows - 1; i++) {
		rightBoundary.push_back(rightBoundary.back() + rightLevelHeightVector);
	}

	// For each row, compute leftToRightUnitVector, outwardNormVector, yawAngle
	for (int i = 0; i < miNumOfRows; i++) {
		mvv3LeftToRightUnitVectors.push_back(
				TooN::unit(rightBoundary[i] - leftBoundary[i]));
		mvv3OutwardNormVectors.push_back(
				TooN::unit(
						TooN::makeVector(0, 0, -1)
								^ mvv3LeftToRightUnitVectors[i]));
		mvdYawAngles.push_back(vectorToYaw(-mvv3OutwardNormVectors[i]));
		cout << "ROW " << i << ": Left " << leftBoundary[i] << "\tRight "
				<< rightBoundary[i] << "\tUnit "
				<< mvv3LeftToRightUnitVectors[i] << "\tOutward "
				<< mvv3OutwardNormVectors[i] << "\tYaw " << mvdYawAngles[i]
				<< endl;
	}

	// row height and col width for each camera view
	mdRowHeight = max(-leftLevelHeightVector[2], -rightLevelHeightVector[2]);
	mdColWidth = mdRowHeight * mdAngleH / mdAngleV;
	cout << "RowHeight:ColWidth\t" << mdRowHeight << ":" << mdColWidth << endl;

	// number of cols
	miNumOfCols = (int) ceil(
			max(sqrt(tEdgeVector * tEdgeVector),
					sqrt(bEdgeVector * bEdgeVector)) / mdColWidth);
	cout << "Num of rows:cols\t" << miNumOfRows << ":" << miNumOfCols << endl;

	// distance to board
	double cotHalfAngleV = cos(mdAngleV / 2 * M_PI / 180)
			/ sin(mdAngleV / 2 * M_PI / 180);
	mdDistToBoard = cotHalfAngleV * mdRowHeight / 2 / 0.6;
	cout << "Distance to board\t" << mdDistToBoard << endl;

	// leftmost and rightmost way points
	vector<TooN::Vector<3> > leftMostWayPoints, rightMostWayPoints;
	for (int i = 0; i < miNumOfRows; i++) {
		leftMostWayPoints.push_back(
				leftBoundary[i] + mdRowHeight / 2 * TooN::makeVector(0, 0, -1)
						+ mdColWidth / 2 * mvv3LeftToRightUnitVectors[i]
						+ mdDistToBoard * mvv3OutwardNormVectors[i]);
		rightMostWayPoints.push_back(
				rightBoundary[i] + mdRowHeight / 2 * TooN::makeVector(0, 0, -1)
						- mdColWidth / 2 * mvv3LeftToRightUnitVectors[i]
						+ mdDistToBoard * mvv3OutwardNormVectors[i]);
	}

	// construct landmarks and waypoints
	mvvv3LandMarks.resize(miNumOfRows);
	mvvv3WayPoints.resize(miNumOfRows);
	if (miNumOfCols == 1) {
		for (int i = 0; i < miNumOfRows; i++) {
			mvvv3LandMarks[i].push_back(leftBoundary[i]);
			mvvv3WayPoints[i].push_back(leftMostWayPoints[i]);
		}
	} else {
		for (int i = 0; i < miNumOfRows; i++) {
			TooN::Vector<3> v3StepSize = (rightMostWayPoints[i]
					- leftMostWayPoints[i]) / (miNumOfCols - 1);
			mvvv3LandMarks[i].push_back(leftBoundary[i]);
			mvvv3WayPoints[i].push_back(leftMostWayPoints[i]);
			for (int j = 1; j < miNumOfCols; j++) {
				mvvv3LandMarks[i].push_back(leftBoundary[i] + v3StepSize * j);
				mvvv3WayPoints[i].push_back(
						leftMostWayPoints[i] + v3StepSize * j);
			}
		}
	}

	cout << "LandMarks\t&\tWaypoints" << endl;
	for (int i = 0; i < miNumOfRows; i++) {
		for (int j = 0; j < miNumOfCols; j++) {
			cout << mvvv3LandMarks[i][j] << "\t" << mvvv3WayPoints[i][j]
					<< endl;
		}
	}

	miCurrentWayPointNum = -1;

	isCompleted = false;
	mpKIHelper = NULL;

	char buf[200];
	snprintf(buf, 200, "zigzagboard");
	command = buf;
}

KIZigZagBoard::~KIZigZagBoard(void) {
}

bool KIZigZagBoard::update(const tum_ardrone::filter_stateConstPtr statePtr) {
	if (isCompleted) {
		mpKIHelper = NULL;
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}

	if (miCurrentWayPointNum < 0) {
		miCurrentWayPointNum = 0;
		miTimer = getMS();
		
		mpKIHelper = new KIFlyAlong(
				DronePosition(TooN::makeVector(statePtr->x, statePtr->y, statePtr->z),
						statePtr->yaw),
				DronePosition(mvvv3WayPoints[0][0],
						mvdYawAngles[0]), min(1.0, mdDistToBoard / 2));   // safe speed
		mpKIHelper->setPointers(this->node, this->controller);
	}

	DronePosition currentPose = DronePosition(
			TooN::makeVector(statePtr->x, statePtr->y, statePtr->z),
			statePtr->yaw);

	// if not reached yet, need to get within small radius to count.
	if (isWayPointReached(miCurrentWayPointNum, currentPose)) {
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
			cout << "Zigzagboard complete!" << endl;
			isCompleted = true;
		} else {
			int oldRowNum = waypointNumToRowNum(miCurrentWayPointNum - 1);
			int oldColNum = waypointNumToColNum(miCurrentWayPointNum - 1);

			int newRowNum = waypointNumToRowNum(miCurrentWayPointNum);
			int newColNum = waypointNumToColNum(miCurrentWayPointNum);
			
			double distance = sqrt((mvvv3WayPoints[oldRowNum][oldColNum] - mvvv3WayPoints[newRowNum][newColNum]) * (mvvv3WayPoints[oldRowNum][oldColNum] - mvvv3WayPoints[newRowNum][newColNum]));
			
			mpKIHelper = new KIFlyAlong(
					DronePosition(mvvv3WayPoints[oldRowNum][oldColNum],
							mvdYawAngles[oldRowNum]),
					DronePosition(mvvv3WayPoints[newRowNum][newColNum],
							mvdYawAngles[newRowNum]), max(0.2, distance / 2)); // speed to overcome drift
		    mpKIHelper->setPointers(this->node, this->controller);
			cout << "Distance to next waypoint is "
					<< mpKIHelper->getDistance()
					<< endl;
		}
	}
	
	mpKIHelper->update(statePtr);
	return false;
	
	// control!
//	node->sendControlToDrone(controller->update(statePtr));
//	return false;	// not done yet (!)
}

