#include "KIRecover.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"

KIRecover::KIRecover(DronePosition checkpointP, double maxTimeS) {
	maxTimeMs = (int) (1000 * maxTimeS);

	checkpoint = checkpointP;

	startAtClock = -1;
	targetSet = false;

//	isCompleted = false;

	char buf[200];
	snprintf(buf, 200, "Recovering %.2f %.2f %.2f %.2f, Time %.2fs",
			checkpointP.pos[0], checkpointP.pos[1], checkpointP.pos[2],
			checkpointP.yaw, maxTimeS);
	command = buf;
}

KIRecover::~KIRecover(void) {
}

bool KIRecover::update(const tum_ardrone::filter_stateConstPtr statePtr) {
	if (!targetSet) {
		controller->setTarget(checkpoint);
		startAtClock = getMS();
	}
	targetSet = true;

	// back to track
	if (statePtr->ptamState == statePtr->PTAM_BEST
			|| statePtr->ptamState == statePtr->PTAM_GOOD
			|| statePtr->ptamState == statePtr->PTAM_TOOKKF) {
		return true;
	}

	// time reached?
	if (getMS() - startAtClock > maxTimeMs) {
		printf("recover failed -> try next recover point\n");
		return true;
	}
//
//	// target reached?
//	if (!isCompleted && reached && (getMS() - reachedAtClock) > stayTimeMs) {
//		printf("checkpoint done!\n");
//		isCompleted = true;
//	}
//	if (isCompleted) {
//		node->sendControlToDrone(controller->update(statePtr));
//		return true;
//	}

	// control!
	node->sendControlToDrone(controller->update(statePtr));
	return false;	// not done yet (!)
}
