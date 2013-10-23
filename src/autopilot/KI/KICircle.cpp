#include "KICircle.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KICircle::KICircle(DronePosition checkpointP, 
		TooN::Vector<3> centerposP,
		TooN::Vector<3> upVectorP,
		double lineSpeedP,
		double stayTime,
		double maxControlFactorP
		)
{
	radius = sqrt((checkpointP.pos - centerposP) * (checkpointP.pos - centerposP));

	startAtClock = -1;
	isCompleted = false;

	stayTimeMs = (int)(1000*stayTime);
	maxControlFactor = maxControlFactorP;

	checkpoint = checkpointP;
	centerpos = centerposP;
	upVector = upVectorP;
	lineSpeed = lineSpeedP;
	
	char buf[200];
	snprintf(buf,200,"circle C_xyz=%.2f %.2f %.2f, Up_vector=%.2f %.2f %.2f, radius=%.2f, v=%.2f, t=%dms", centerpos[0], centerpos[1], centerpos[2], upVector[0], upVector[1], upVector[2], radius, lineSpeedP, stayTimeMs);
	command = buf;
}


KICircle::~KICircle(void)
{
}


bool KICircle::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	// time reached?
	if(!isCompleted && startAtClock >= 0 && ((getMS() - startAtClock) > stayTimeMs))
	{
		controller->clearDirection();
		printf("circle done!\n");
		isCompleted = true;
	}
	if(isCompleted)
	{
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}

	// get diffVector
	TooN::Vector<3> diffVector = TooN::makeVector(
				statePtr->x - centerpos[0],
				statePtr->y - centerpos[1],
				statePtr->z - centerpos[2]);

	// set direction and lineSpeed
	TooN::Vector<3> dirVector = diffVector ^ upVector;	// vector of direction = diff x up
	controller->setDirection(DronePosition(dirVector, 0.0), lineSpeed);
	
	// compute target pos
	TooN::Vector<3> radiusVector = upVector ^ dirVector;	// vector away from centre = up x direction
	radiusVector /= sqrt(radiusVector * radiusVector);
	radiusVector *= radius;
	checkpoint.pos = centerpos + radiusVector;
	
	// compute target yaw
	if(abs(diffVector[0]) < 0.1)
	{
		checkpoint.yaw = 0;
	} 
	else 
	{
		// arccos ( -diff_xy . (0,1) / |-diff_xy| * |(0,1)| )
		checkpoint.yaw = 180.0/3.1416 * acos(-diffVector[1]/sqrt(diffVector[0] * diffVector[0] + diffVector[1] * diffVector[1]));

		// check x value in diff to determine sign
		if(diffVector[0] > 0)
		{
			checkpoint.yaw *= -1;
		}
	}
	// set target
	controller->setTarget(checkpoint);
	
	if (startAtClock < 0)
	{
		startAtClock = getMS();
		printf("start moving along a circle.\n");
	}

	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}
