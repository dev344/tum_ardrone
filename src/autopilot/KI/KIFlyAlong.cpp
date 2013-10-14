#include "KIFlyAlong.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KIFlyAlong::KIFlyAlong(DronePosition checkpointP, 
		DronePosition directionP,
		double lineSpeedP,
		double stayTime,
		double maxControlFactorP
		)
{
	stayTimeMs = (int)(1000*stayTime);
	maxControlFactor = maxControlFactorP;

	checkpoint = checkpointP;
	direction = directionP;
	lineSpeed = lineSpeedP;

	startAtClock = -1;
	directionSet = false;
	isCompleted = false;

	char buf[200];
	snprintf(buf,200,"goAlong %.2f %.2f %.2f %.2f, v=%.2f, t=%dms", directionP.pos[0], directionP.pos[1], directionP.pos[2], directionP.yaw, lineSpeedP, stayTimeMs);
	command = buf;
}


KIFlyAlong::~KIFlyAlong(void)
{
}


bool KIFlyAlong::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	// set direction and lineSpeed
	if(!directionSet)
	{
		controller->setDirection(direction, lineSpeed);
	}
	directionSet = true;

	// time reached?
	if(!isCompleted && startAtClock >= 0 && ((getMS() - startAtClock) > stayTimeMs))
	{
		printf("line done!\n");
		isCompleted = true;
	}
	if(isCompleted)
	{
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}

	// set target
	if(direction.pos * direction.pos > 0.01)
	{
		TooN::Vector<3> diffs = TooN::makeVector(
				statePtr->x - checkpoint.pos[0],
				statePtr->y - checkpoint.pos[1],
				statePtr->z - checkpoint.pos[2]);
		checkpoint.pos += direction.pos * ((diffs * direction.pos) / (direction.pos * direction.pos));	// checkpoint update
	}
	checkpoint.yaw += direction.yaw;
	controller->setTarget(checkpoint);
	
	if (startAtClock < 0)
	{
		startAtClock = getMS();
		printf("start moving along a line.\n");
	}

	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}
