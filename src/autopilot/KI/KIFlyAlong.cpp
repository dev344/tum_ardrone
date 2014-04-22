#include "KIFlyAlong.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KIFlyAlong::KIFlyAlong(DronePosition startPositionP,
		TooN::Vector<3> directionP,
		double lineSpeedP,
		double distanceP
		)
{
	startPosition = startPositionP;
	checkpoint = startPositionP.pos;
	direction = TooN::unit(directionP);
	lineSpeed = lineSpeedP;
	distance = distanceP;
	directionSet = false;
	isCompleted = false;

	char buf[200];
	snprintf(buf,200,"goAlong %.2f %.2f %.2f, v=%.2f, distance=%.2fm", directionP[0], directionP[1], directionP[2], lineSpeedP, distanceP);
	command = buf;
}


KIFlyAlong::~KIFlyAlong(void)
{
}


bool KIFlyAlong::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	if(isCompleted)
	{
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}

	// get current position
	TooN::Vector<3> currentpoint = TooN::makeVector(statePtr->x, statePtr->y, statePtr->z);
	
	// distance reached?
	TooN::Vector<3> diffs = currentpoint - startPosition.pos;
	if ((diffs * direction) / (direction * direction) >= distance)
	{
		//controller->clearDirection();
		printf("line done!\n");
		isCompleted = true;
		return false;
	}

	// set target
	diffs = currentpoint - checkpoint;
	checkpoint += direction * ((diffs * direction) / (direction * direction));	// checkpoint update	
	controller->setTarget(DronePosition(checkpoint + lineSpeed * direction, startPosition.yaw));

	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}
