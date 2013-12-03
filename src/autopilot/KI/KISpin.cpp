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
	snprintf(buf,200,"spin v=%.2f, d=%.2fdegree", spinSpeedP, distanceP);
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

	// checkpoint reached?
	double diff = startPosition.yaw + checkpoint - statePtr->yaw;
	while(diff < -180) diff += 360;
	while(diff >=  180) diff -= 360;
	if (fabs(diff) < std::min(90.0, 10*fabs(spinSpeed)))
	{
		// distance reached?
		if (fabs(checkpoint) >= distance)
		{
			printf("spin done!\n");
			isCompleted = true;
			return false;
		}
		checkpoint += spinSpeed;
	}

	// set target
	controller->setTarget(DronePosition(startPosition.pos, startPosition.yaw + checkpoint));


	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}
