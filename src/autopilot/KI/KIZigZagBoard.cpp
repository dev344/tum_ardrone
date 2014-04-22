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

	cout << "BOARD CORNERS" << endl << mv3TopLPoint << "\t" << mv3TopRPoint << endl << mv3BtmLPoint << "\t" << mv3BtmRPoint << endl;
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

	mv3OutwardNormVector = TooN::unit((tlNormVector + blNormVector + brNormVector
			+ trNormVector) / 4.0);

	// yaw angle direction = arccos ( -ourwardNorm_xy . (0,1) / |-ourwardNorm_xy| * |(0,1)| )
	mdYawAngle =
			180.0 / M_PI
					* acos(
							-mv3OutwardNormVector[1]
									/ sqrt(
											mv3OutwardNormVector.slice<0, 2>()
													* mv3OutwardNormVector.slice<
															0, 2>()));

	// left and right boundaries
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

	// leftmost and rightmost way points distance to the board plane
	double cotHalfAngleV = cos(mdAngleV / 2 * M_PI / 180)
			/ sin(mdAngleV / 2 * M_PI / 180);

	double leftLevelHeight = sqrt(
			leftLevelHeightVector * leftLevelHeightVector);
	double rightLevelHeight = sqrt(
			rightLevelHeightVector * rightLevelHeightVector);

	double leftDistanceToBoard = max(
			3.0 / 4.0 * leftLevelHeight * cotHalfAngleV, 1.0);
	double rightDistanceToBoard = max(
			3.0 / 4.0 * rightLevelHeight * cotHalfAngleV, 1.0);

	// leftmost and rightmost way points distance to the left/right boundary
	double tanHalfAngleH = sin(mdAngleH / 2 * M_PI / 180)
			/ cos(mdAngleH / 2 * M_PI / 180);

	double leftDistanceToLeftBoundary = 2.0 / 3.0 * leftDistanceToBoard
			* tanHalfAngleH;
	double rightDistanceToRightBoundary = 2.0 / 3.0 * rightDistanceToBoard
			* tanHalfAngleH;

	// leftmost and rightmost way points
	vector<TooN::Vector<3> > leftMostWayPoints, rightMostWayPoints,
			leftToRightUnitVectors;
	for (int i = 0; i < miNumOfRows; i++) {
		leftToRightUnitVectors.push_back(
				TooN::unit(rightBoundary[i] - leftBoundary[i]));
		leftMostWayPoints.push_back(
				leftBoundary[i] + leftLevelHeightVector / 2
						+ leftDistanceToLeftBoundary
								* leftToRightUnitVectors.back()
						+ leftDistanceToBoard * mv3OutwardNormVector);
		rightMostWayPoints.push_back(
				rightBoundary[i] + rightLevelHeightVector / 2
						- rightDistanceToRightBoundary
								* leftToRightUnitVectors.back()
						+ rightDistanceToBoard * mv3OutwardNormVector);
	}

	// approx # of way point
	double leftToRightLength = sqrt(tEdgeVector * tEdgeVector);
	miNumOfCols = (int) ceil(
			leftToRightLength / (leftDistanceToLeftBoundary * 2));

	// ratio
	double ratio = pow(
			rightDistanceToRightBoundary / leftDistanceToLeftBoundary,
			1.0 / (miNumOfCols - 1));

	// set way points
	mvvv3WayPoints.resize(miNumOfRows);
	for (int i = 0; i < miNumOfRows; i++) {
		mvvv3WayPoints[i].resize(miNumOfCols);
		// leftmost and rightmost
		mvvv3WayPoints[i][0] = leftMostWayPoints[i];
		mvvv3WayPoints[i][miNumOfCols - 1] = rightMostWayPoints[i];
		// the rest
		for (int j = 1; j < miNumOfCols - 1; j++) {
			mvvv3WayPoints[i][j] = leftMostWayPoints[i]
					+ leftToRightUnitVectors[i] * leftDistanceToLeftBoundary
							* (ratio + 1) * pow(ratio, j - 1);
		}
	}

	for (int i = 0; i < miNumOfRows; i++) {
		for (int j = 0; j < miNumOfCols; j++) {
			if (i % 2 == 0) {
				qPoses.push(DronePosition(mvvv3WayPoints[i][j], mdYawAngle));
			} else {
				qPoses.push(
						DronePosition(mvvv3WayPoints[i][miNumOfCols - 1 - j],
								mdYawAngle));
			}
			cout << qPoses.back().pos << "\t";
		}
	}

	stayTimeMs = 500;
	initialReachedDist = 0.2;
	stayWithinDist = 0.5;

//	checkpoint = checkpointP;

	reachedAtClock = -1;
	reached = false;

	isStarted = false;
	isCompleted = false;

	char buf[200];
	snprintf(buf, 200, "zigzagboard");
	command = buf;
}

KIZigZagBoard::~KIZigZagBoard(void) {
}

bool KIZigZagBoard::update(const tum_ardrone::filter_stateConstPtr statePtr) {
	if (!isStarted) {
		checkpoint = qPoses.front();
		controller->setTarget(checkpoint);
	}
	isStarted = true;

	// checkpoint reached?
	if (!isCompleted && reached && (getMS() - reachedAtClock) > stayTimeMs) {
		printf("checkpoint done!\n");
		qPoses.pop();
		if (qPoses.empty()) {
			printf("zigzag done!\n");
			isCompleted = true;
		} else {
			checkpoint = qPoses.front();
			controller->setTarget(checkpoint);
			reached = false;
			reachedAtClock = getMS();
		}
	}
	if (isCompleted) {
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}

	// get target dist:
	TooN::Vector<3> diffs = TooN::makeVector(statePtr->x - checkpoint.pos[0],
			statePtr->y - checkpoint.pos[1], statePtr->z - checkpoint.pos[2]);

	double diffYaw = statePtr->yaw - checkpoint.yaw;
	double diffDistSquared = diffs * diffs;

	// if not reached yet, need to get within small radius to count.
	if (!reached && diffDistSquared < initialReachedDist * initialReachedDist
			&& diffYaw * diffYaw < 25) {
		reached = true;
		reachedAtClock = getMS();
		printf("target reached initially!\n");
	}

	// if too far away again: revoke reached status...
	if (reached
			&& (diffDistSquared > stayWithinDist * stayWithinDist
					|| diffYaw * diffYaw > 25)) {
		reached = false;
		printf("target lost again!\n");
	}

	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}

