#include "KISpin.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KISpin::KISpin(DronePosition startPositionP,
		double spinSpeedP,
		double distanceP
		)
{
	startPosition = startPositionP;
	spinSpeed = spinSpeedP;
	distance = distanceP;

	checkpoint = 0;

	spinSet = false;
	isCompleted = false;

	char buf[200];
	snprintf(buf,200,"spin %.2f, distance=%.2fdegree", spinSpeedP, distanceP);
	command = buf;
}


KISpin::~KISpin(void)
{
}


bool KISpin::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	if(isCompleted)
	{
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}

	// set direction and lineSpeed
	if(!spinSet)
	{
		controller->setSpinSpeed(spinSpeed);
	}
	spinSet = true;
	
	// distance reached?
	if (checkpoint >= distance)
	{
		controller->clearSpin();
		printf("spin done!\n");
		isCompleted = true;
		return false;
	}

	// set target
	controller->setTarget(DronePosition(startPosition.pos, startPosition.yaw + checkpoint));

	// checkpoint reached?
	int diff = startPosition.yaw + checkpoint - statePtr->yaw;
	while(diff < -180) diff += 360;
	while(diff >=  180) diff -= 360;
	
	if (abs(diff) < 1)
	{
		checkpoint += spinSpeed;
	}
	
	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}
