#include "KIFlyAlong.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"

ControlCommand KIFlyAlong::ctrlCmdAlongDirection(TooN::Vector<3> error, double yaw) {
	double yawRad = yaw * M_PI / 180;
	ControlCommand result;
	if (sqrt(error * error) < mdLinearSpeed) {
		result.roll = (error[0] * cos(yawRad) - error[1] * sin(yawRad));
		result.pitch = (-error[0] * sin(yawRad) - error[1] * cos(yawRad));
		result.gaz = error[2];
		result.yaw = 0;
	} else {
		result.roll = mdLinearSpeed
				* (mv3DirectionUnitVector[0] * cos(yawRad)
						- mv3DirectionUnitVector[1] * sin(yawRad));
		result.pitch = mdLinearSpeed
				* (-mv3DirectionUnitVector[0] * sin(yawRad)
						- mv3DirectionUnitVector[1] * cos(yawRad));
		result.gaz = mdLinearSpeed * (mv3DirectionUnitVector[2]);
		result.yaw = 0;
	}
	return result;
}

KIFlyAlong::KIFlyAlong(DronePosition startPose, DronePosition endPose,
		double linearSpeed) {
	mposeStart = startPose;
	mposeEnd = endPose;
	mdLinearSpeed = linearSpeed;

	mv3DirectionUnitVector = TooN::unit(mposeEnd.pos - mposeStart.pos);
	mdDistance = sqrt(
			(mposeEnd.pos - mposeStart.pos) * (mposeEnd.pos - mposeStart.pos));
	mdAngleDistance = angleToValidYaw(endPose.yaw - startPose.yaw);

	isCompleted = false;

	char buf[200];
	snprintf(buf, 200,
			"goAlong from %.2f %.2f %.2f (%.2f) to %.2f %.2f %.2f (%.2f), speed=%.2f",
			mposeStart.pos[0], mposeStart.pos[1], mposeStart.pos[2],
			mposeStart.yaw, mposeEnd.pos[0], mposeEnd.pos[1], mposeEnd.pos[2],
			mposeEnd.yaw, mdLinearSpeed);
	command = buf;

	cout << command << endl;
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

	ControlCommand ctrlcmd;

	// terminal condition
	if (v * u >= mdDistance) {
		controller->setTarget(mposeEnd, true);
		ctrlcmd = controller->update(statePtr);
		cout << "FlyAlong done!" << endl;
		isCompleted = true;
	}
	// special case around start pose
	else if (v * u <= 0) {
		controller->setTarget(
		        DronePosition(mposeStart.pos + puv, mposeStart.yaw), true);
		ControlCommand pidCmd = controller->update(statePtr);
		ControlCommand dirCmd = ctrlCmdAlongDirection(
				mposeEnd.pos - mposeStart.pos, statePtr->yaw);
		ctrlcmd = pidCmd + dirCmd;
	}
	// normal case
	else {
		controller->setTarget(
				DronePosition(mposeStart.pos + puv, mposeStart.yaw + (v * u) / mdDistance * mdAngleDistance), true);
		ControlCommand pidCmd = controller->update(statePtr);
		ControlCommand dirCmd = ctrlCmdAlongDirection(
				mposeEnd.pos - (mposeStart.pos + puv), statePtr->yaw);
		ctrlcmd = pidCmd + dirCmd;
		//cout << "PID: roll " << pidCmd.roll << "\tpitch " << pidCmd.pitch << "\tgaz " << pidCmd.gaz << "\tyaw " << pidCmd.yaw << endl;
		//cout << "DIR: roll " << dirCmd.roll << "\tpitch " << dirCmd.pitch << "\tgaz " << dirCmd.gaz << "\tyaw " << dirCmd.yaw << endl;
		//cout << "CTRL: roll " << ctrlcmd.roll << "\tpitch " << ctrlcmd.pitch << "\tgaz " << ctrlcmd.gaz << "\tyaw " << ctrlcmd.yaw << endl;
	}

	// control!
	node->sendControlToDrone(ctrlcmd);
	return false;	// not done yet (!)
}
