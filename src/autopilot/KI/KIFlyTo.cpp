/**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "KIFlyTo.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"

KIFlyTo::KIFlyTo(DronePosition checkpointP, double stayTime,
		double maxControlFactorP, double initialReachedDistP,
		double stayWithinDistP, bool isLoggingP) {
	stayTimeMs = (int) (1000 * stayTime);
	maxControlFactor = maxControlFactorP;
	initialReachedDist = initialReachedDistP;
	stayWithinDist = stayWithinDistP;

	checkpoint = checkpointP;

	reachedAtClock = -1;
	reached = false;

	targetSet = false;

	isCompleted = false;

	isLogging = isLoggingP;
	logFileName = "PTAMHovering.csv";
	logFile.open(logFileName.c_str());
	if (logFile.is_open()) {
		stringstream saLine;
		saLine
				<< "time,x_x,x_y,x_z,x_roll,x_pitch,x_yaw,x_dx,x_dy,x_dz,x_dyaw,x_battery,u_roll,u_pitch,u_yaw,u_gaz\n";
		logFile << saLine.str();
	} else {
		printf("create PTAMHovering.csv failed!\n");
	}

	char buf[200];
	snprintf(buf, 200, "goto %.2f %.2f %.2f %.2f", checkpointP.pos[0],
			checkpointP.pos[1], checkpointP.pos[2], checkpointP.yaw);
	command = buf;
}

KIFlyTo::~KIFlyTo(void) {
}

bool KIFlyTo::update(const tum_ardrone::filter_stateConstPtr statePtr) {
	if (!targetSet)
		controller->setTarget(checkpoint);
	targetSet = true;

	// target reached?
	if (!isCompleted && reached && (getMS() - reachedAtClock) > stayTimeMs) {
		printf("checkpoint done!\n");
		isCompleted = true;
	}
	if (isCompleted) {
		node->sendControlToDrone(controller->update(statePtr));
		return true;
	}

	// get target dist:
	TooN::Vector<3> diffs = TooN::makeVector(statePtr->x - checkpoint.pos[0],
			statePtr->y - checkpoint.pos[1], statePtr->z - checkpoint.pos[2]);

	double diffYaw = statePtr->yaw - checkpoint.yaw;
	double diffDistSquared = diffs[0] * diffs[0] + diffs[1] * diffs[1]
			+ diffs[2] * diffs[2];

	// if not reached yet, need to get within small radius to count.
	if (!reached && diffDistSquared < initialReachedDist * initialReachedDist
			&& diffYaw * diffYaw < 25) {
		reached = true;
		reachedAtClock = getMS();
		printf("target reached initially!\n");
	}

	// if too far away again: revoke reached status...
	if (reached
			&& (diffDistSquared > stayWithinDist * stayWithinDist
					|| diffYaw * diffYaw > 25)) {
		reached = false;
		printf("target lost again!\n");
	}

	// control!
	ControlCommand u = controller->update(statePtr);
	if (isLogging) {
		if (fabs(u.gaz) <= 0.2) {
			u.gaz = 0;
		} else if (u.gaz > 0) {
			u.gaz = 1;
		} else {
			u.gaz = -1;
		}

		if (fabs(u.pitch) <= 0.2) {
			u.pitch = 0;
		} else if (u.pitch > 0) {
			u.pitch = 1;
		} else {
			u.pitch = -1;
		}

		if (fabs(u.roll) <= 0.2) {
			u.roll = 0;
		} else if (u.roll > 0) {
			u.roll = 1;
		} else {
			u.roll = -1;
		}

		if (fabs(u.yaw) <= 0.2) {
			u.yaw = 0;
		} else if (u.yaw > 0) {
			u.yaw = 1;
		} else {
			u.yaw = -1;
		}

		if (logFile.is_open()) {
			stringstream saLine;
			saLine << getMS() << "," << statePtr->x << "," << statePtr->y << ","
					<< statePtr->z << "," << statePtr->roll << ","
					<< statePtr->pitch << "," << statePtr->yaw << ","
					<< statePtr->dx << "," << statePtr->dy << ","
					<< statePtr->dz << "," << statePtr->dyaw << ","
					<< statePtr->batteryPercent << "," << u.roll << ","
					<< u.pitch << "," << u.yaw << "," << u.gaz;
			logFile << saLine.str() << "\n";
		}
	}
	node->sendControlToDrone(u);
	return false;	// not done yet (!)
}
