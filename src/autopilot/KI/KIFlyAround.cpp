#include "KIFlyAround.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"

ControlCommand KIFlyAround::ctrlCmdAlongCircle(TooN::Vector<3> directionUnitVector,
		double checkpointYaw, double currentYaw) {
	// cout << "ApproxDistanceToGoal: " << mdApproxDistance << endl;

	double yawRad = currentYaw * M_PI / 180;
	ControlCommand result;
	if (fabs(mdApproxDistance) < mdLinearSpeed) {
		result.roll = mdApproxDistance
				* (directionUnitVector[0] * cos(yawRad)
						- directionUnitVector[1] * sin(yawRad));
		result.pitch = mdApproxDistance
				* (-directionUnitVector[0] * sin(yawRad)
						- directionUnitVector[1] * cos(yawRad));
		result.gaz = mdApproxDistance * directionUnitVector[2];
		result.yaw = checkpointYaw - currentYaw;
	} else {
		result.roll = mdLinearSpeed
				* (directionUnitVector[0] * cos(yawRad)
						- directionUnitVector[1] * sin(yawRad))
				* (mdApproxDistance > 0 ? 1 : -1);
		result.pitch = mdLinearSpeed
				* (-directionUnitVector[0] * sin(yawRad)
						- directionUnitVector[1] * cos(yawRad))
				* (mdApproxDistance > 0 ? 1 : -1);
		result.gaz = mdLinearSpeed * directionUnitVector[2]
				* (mdApproxDistance > 0 ? 1 : -1);
		result.yaw = checkpointYaw - currentYaw;
	}
	return result;
}

bool KIFlyAround::isTotalYawAngleAchieved(double checkpointYaw) {
	checkpointYaw = smallestAngleToTarget(checkpointYaw, mdYawAngleCheckPoint);
	// decreasing yaw angle
	if (mv3CirclePlaneNormVector[2] > 0) {
		mdApproxDistance = mdRadius
				* (mdStartYawAngle - mdTotalYawAngle - checkpointYaw) * M_PI / 180; // distance could be +/-
		if (checkpointYaw < mdYawAngleCheckPoint && checkpointYaw > mdYawAngleCheckPoint - 90) {
			mdYawAngleCheckPoint = checkpointYaw;	// update yawAngleCheckPoint
		}
		if (mdYawAngleCheckPoint < mdStartYawAngle - mdTotalYawAngle) {
			return true;
		}
	}
	// increasing yaw angle
	else {
		mdApproxDistance = mdRadius
				* (mdStartYawAngle + mdTotalYawAngle - checkpointYaw) * M_PI / 180; // distance could be +/-
		if (checkpointYaw > mdYawAngleCheckPoint && checkpointYaw < mdYawAngleCheckPoint + 90) {
			mdYawAngleCheckPoint = checkpointYaw;	// update yawAngleCheckPoint
		}
		if (mdYawAngleCheckPoint > mdStartYawAngle + mdTotalYawAngle) {
			return true;
		}
	}
	return false;
}

KIFlyAround::KIFlyAround(TooN::Vector<3> centerPoint, double radius,
		double startYawAngle, double totalYawAngle,
		TooN::Vector<3> circlePlaneNormVector, double linearSpeed) {
	mv3CenterPoint = centerPoint;
	mdRadius = radius;
	mdStartYawAngle = startYawAngle;
	mdTotalYawAngle = totalYawAngle;
	mv3CirclePlaneNormVector = circlePlaneNormVector;
	mdLinearSpeed = linearSpeed;

	mdYawAngleCheckPoint = startYawAngle;

	isCompleted = false;

	char buf[200];
	snprintf(buf, 200,
			"circle Center=%.2f %.2f %.2f, Radius=%.2f, angle=%.2f from %.2f, norm=%.2f %.2f %.2f, v=%.2f",
			mv3CenterPoint[0], mv3CenterPoint[1], mv3CenterPoint[2], mdRadius,
			mdTotalYawAngle, mdStartYawAngle, mv3CirclePlaneNormVector[0],
			mv3CirclePlaneNormVector[1], mv3CirclePlaneNormVector[2],
			mdLinearSpeed);
	command = buf;
}

KIFlyAround::~KIFlyAround(void) {
}

bool KIFlyAround::update(const tum_ardrone::filter_stateConstPtr statePtr) {
	if (isCompleted) {
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}

	TooN::Vector<3> v3CurrentPoint = TooN::makeVector(statePtr->x, statePtr->y,
			statePtr->z);
	TooN::Vector<3> v3CurrentToCenter = mv3CenterPoint - v3CurrentPoint;
	TooN::Vector<3> v3DirectionUnitVector = TooN::unit(
			v3CurrentToCenter ^ mv3CirclePlaneNormVector);
	TooN::Vector<3> v3CenterToCheckPoint = mdRadius
			* TooN::unit(v3DirectionUnitVector ^ mv3CirclePlaneNormVector);
	TooN::Vector<3> v3CheckPoint = mv3CenterPoint + v3CenterToCheckPoint;

	double checkpointYaw = vectorToYaw(-v3CenterToCheckPoint);

	ControlCommand ctrlcmd;

	// terminal condition
	if (isTotalYawAngleAchieved(checkpointYaw)) {
		controller->setTarget(DronePosition(v3CheckPoint, checkpointYaw), true);
		ctrlcmd = controller->update(statePtr);
		cout << "FlyAround done!" << endl;
		isCompleted = true;
	}
	// normal case
	else {
		controller->setTarget(DronePosition(v3CheckPoint, checkpointYaw/*statePtr->yaw*/), true);
		ControlCommand pidCmd = controller->update(statePtr);
		ControlCommand dirCmd = ctrlCmdAlongCircle(v3DirectionUnitVector,
				/*checkpointYaw*/statePtr->yaw, statePtr->yaw);
		ctrlcmd = pidCmd + dirCmd;
	}

	// control!
	node->sendControlToDrone(ctrlcmd);
	return false;	// not done yet (!)
}
