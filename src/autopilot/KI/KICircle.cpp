#include "KICircle.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KICircle::KICircle(DronePosition startPositionP, 
		TooN::Vector<3> centerPointP,
		TooN::Vector<3> upVectorP,
		double lineSpeedP,
		double angleP
		)
{
	radius = sqrt((startPositionP.pos - centerPointP) * (startPositionP.pos - centerPointP));

	isCompleted = false;

	startPosition = startPositionP;
	checkPosition = startPositionP;
	centerPoint = centerPointP;
	upVector = upVectorP;
	lineSpeed = lineSpeedP;
	targetAngle = startPositionP.yaw + angleP;
	while(angleP < -180) angleP += 360;
	while(angleP >=  180) angleP -= 360;
	while(targetAngle < -180) targetAngle += 360;
	while(targetAngle >=  180) targetAngle -= 360;
	
	
	char buf[200];
	snprintf(buf,200,"circle Center=%.2f %.2f %.2f, UpVector=%.2f %.2f %.2f, radius=%.2f, v=%.2f, angle=%ddegree", centerPoint[0], centerPoint[1], centerPoint[2], upVector[0], upVector[1], upVector[2], radius, lineSpeedP, angleP);
	command = buf;
}


KICircle::~KICircle(void)
{
}


bool KICircle::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	if(isCompleted)
	{
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}

	// distance reached?
	if((targetAngle - statePtr->yaw) * (targetAngle - statePtr->yaw) < 10)
	{
		printf("circle done!\n");
		isCompleted = true;
	}

	// get current point
	TooN::Vector<3> currentPoint = TooN::makeVector(statePtr->x, statePtr->y, statePtr->z);

	// get diffVector
	TooN::Vector<3> diffVector = currentPoint - centerPoint;

	// get dirVector
	TooN::Vector<3> dirVector = unifyVector(diffVector ^ upVector);	// vector of direction = diff x up

	// get radiusVector
	TooN::Vector<3> radiusVector = unifyVector(upVector ^ dirVector);	// vector away from centre = up x direction
	radiusVector *= radius;
	
	// compute target position
	checkPosition.pos = centerPoint + radiusVector + lineSpeed * dirVector;
	if(sqrt(diffVector * diffVector) < 0.5)	// too near
	{
		printf("no change to yaw because of too near distance: %f\n", sqrt(diffVector * diffVector));
	}
	else 
	{
		// arccos ( -diff_xy . (0,1) / |-diff_xy| * |(0,1)| )
		checkPosition.yaw = 180.0/3.1416 * acos(-diffVector[1]/sqrt(diffVector[0] * diffVector[0] + diffVector[1] * diffVector[1]));

		// check x value in diff to determine sign
		if(diffVector[0] > 0)
		{
			checkPosition.yaw *= -1;
		}
	}

	// set target
	controller->setTarget(checkPosition);
	
	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}
