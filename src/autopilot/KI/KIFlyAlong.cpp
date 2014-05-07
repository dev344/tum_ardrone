#include "KIFlyAlong.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"

KIFlyAlong::KIFlyAlong(DronePosition startPose, DronePosition endPose,
		double linearSpeed) {
	mposeStart = startPose;
	while (mposeStart.yaw < 0) {
		mposeStart.yaw += 360;
	}
	mposeEnd = endPose;
	while (mposeEnd.yaw < 0) {
		mposeEnd.yaw += 360;
	}
	mdLinearSpeed = linearSpeed;

	mv3DirectionUnitVector = TooN::unit(mposeEnd.pos - mposeStart.pos);
	mdDistance = sqrt(
			(mposeEnd.pos - mposeStart.pos) * (mposeEnd.pos - mposeStart.pos));

	isCompleted = false;

	char buf[200];
	snprintf(buf, 200,
			"goAlong from %.2f %.2f %.2f (%.2f) to %.2f %.2f %.2f (%.2f), speed=%.2f",
			mposeStart.pos[0], mposeStart.pos[1], mposeStart.pos[2],
			mposeStart.yaw, mposeEnd.pos[0], mposeEnd.pos[1], mposeEnd.pos[2],
			mposeEnd.yaw, mdLinearSpeed);
	command = buf;
}

KIFlyAlong::~KIFlyAlong(void) {
}

bool KIFlyAlong::update(const tum_ardrone::filter_stateConstPtr statePtr) {
	if (isCompleted) {
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}

	// http://www.lighthouse3d.com/tutorials/maths/vector-projection/
	TooN::Vector<3> u = TooN::makeVector(statePtr->x, statePtr->y, statePtr->z)
			- mposeStart.pos;
	TooN::Vector<3> v = mv3DirectionUnitVector;
	TooN::Vector<3> puv = (v * u) * v;	// project u onto v

	// special case around start pose
	if (v * u <= 0) {
		controller->setTarget(
				DronePosition(
						mposeStart.pos + puv
								+ mdLinearSpeed * mv3DirectionUnitVector,
						mposeStart.yaw), true);
	}

	// special case around end pose
	if ((v * u) + mdLinearSpeed >= mdDistance) {
		controller->setTarget(mposeEnd);
		cout << "flyAlong done!" << endl;
		isCompleted = true;
	}

	// normal case
	controller->setTarget(
			DronePosition(
					mposeStart.pos + puv
							+ mdLinearSpeed * mv3DirectionUnitVector,
					mposeStart.yaw
							+ ((v * u) + mdLinearSpeed) / mdDistance
									* (mposeEnd.yaw - mposeStart.yaw)), true);

	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}
