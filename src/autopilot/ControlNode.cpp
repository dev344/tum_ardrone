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

#include "ControlNode.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/package.h>

#include "geometry_msgs/Twist.h"
#include "../HelperFunctions.h"
#include "tum_ardrone/filter_state.h"
#include <ardrone_autonomy/Navdata.h>
#include "std_msgs/String.h"
#include <sys/stat.h>
#include <string>

// include KI's
#include "KI/KIAutoInit.h"
#include "KI/KIFlyTo.h"
#include "KI/KILand.h"
#include "KI/KIFlyAlong.h"	//[ziquan]
#include "KI/KISpin.h"		//[ziquan]
#include "KI/KIFlyAround.h"	//[ziquan]
#include "KI/KIQLearning.h"	//[ziquan]
#include "KI/KIModel.h"		//[ziquan]
#include "KI/KIRepsExe.h"	//[ziquan]
#include "KI/KIRecover.h"	//[ziquan]
#include "KI/KIZigZagBoard.h"	//[ziquan]
#include "KI/KIZigZagTower.h"	//[ziquan]
#include "KI/KIProcedure.h"

// [Devesh]
//#include <cmath>
//#define PI 3.14159265
#include "KI/KIFlyToRelDir.h"

using namespace std;

pthread_mutex_t ControlNode::logControl_CS = PTHREAD_MUTEX_INITIALIZER;

ControlNode::ControlNode() {
	control_channel = nh_.resolveName("cmd_vel");
	dronepose_channel = nh_.resolveName("ardrone/predictedPose");
	navdata_channel = nh_.resolveName("ardrone/navdata");
	command_channel = nh_.resolveName("tum_ardrone/com");
	takeoff_channel = nh_.resolveName("ardrone/takeoff");
	land_channel = nh_.resolveName("ardrone/land");
	toggleState_channel = nh_.resolveName("ardrone/reset");

	packagePath = ros::package::getPath("tum_ardrone");

	std::string val;
	float valFloat;

	ros::param::get("~minPublishFreq", val);
	if (val.size() > 0)
		sscanf(val.c_str(), "%f", &valFloat);
	else
		valFloat = 110;
	minPublishFreq = valFloat;
	cout << "set minPublishFreq to " << valFloat << "ms" << endl;

	// other internal vars
	logfileControl = 0;
	hoverCommand.gaz = hoverCommand.pitch = hoverCommand.roll =
			hoverCommand.yaw = 0;
	lastControlSentMS = 0;

	// channels
	dronepose_sub = nh_.subscribe(dronepose_channel, 10,
			&ControlNode::droneposeCb, this);
	navdata_sub = nh_.subscribe(navdata_channel, 10, &ControlNode::navdataCb,
			this);
	vel_pub = nh_.advertise<geometry_msgs::Twist>(control_channel, 1);
	tum_ardrone_pub = nh_.advertise<std_msgs::String>(command_channel, 50);
	tum_ardrone_sub = nh_.subscribe(command_channel, 50, &ControlNode::comCb,
			this);
	takeoff_pub = nh_.advertise<std_msgs::Empty>(takeoff_channel, 1);
	land_pub = nh_.advertise<std_msgs::Empty>(land_channel, 1);
	toggleState_pub = nh_.advertise<std_msgs::Empty>(toggleState_channel, 1);

	// [Devesh]
	interface_channel = nh_.resolveName("meneldor/ui_com");
	interface_directions_pub = nh_.advertise<std_msgs::String>(
			interface_channel, 50);

	// internals
	parameter_referenceZero = DronePosition(TooN::makeVector(0, 0, 0), 0);
	parameter_MaxControl = 1;
	parameter_InitialReachDist = 0.2;
	parameter_StayWithinDist = 0.5;
	parameter_StayTime = 2;
	parameter_LineSpeed = 0.1;
	parameter_GSVScalar = 1.8;
	isControlling = false;
	currentKI = NULL;
	lastSentControl = ControlCommand(0, 0, 0, 0);

	// create controller
	controller = DroneController();
	controller.node = this;
	// [ziquan] trajectory
	trajectoryMaxSize = 300;

	// [ziquan] force keyframe
	lastForceKFMS = -1;
}

ControlNode::~ControlNode() {

}

pthread_mutex_t ControlNode::commandQueue_CS = PTHREAD_MUTEX_INITIALIZER;
void ControlNode::droneposeCb(
		const tum_ardrone::filter_stateConstPtr statePtr) {
	// do controlling
	pthread_mutex_lock(&commandQueue_CS);

	/*
	 <<<<<<< HEAD
	 */
	// as long as no KI present:
	// pop next KI (if next KI present).
	while (currentKI == NULL && commandQueue.size() > 0)
		popNextCommand(statePtr);

	// if there is no current KI now, we obviously have no current goal -> send drone hover
	if (currentKI != NULL) {
		// let current KI control.
		if (currentKI->update(statePtr) && commandQueue.size() > 0) {
			delete currentKI;
			currentKI = NULL;
		}
	} else if (isControlling) {
		sendControlToDrone(hoverCommand);
		ROS_WARN(
				"Autopilot is Controlling, but there is no KI -> sending HOVER");
	}

	pthread_mutex_unlock(&commandQueue_CS);
	/*
	 =======
	 // nasty force keyframe every 1 second
	 if (lastForceKFMS < 0)
	 {
	 lastForceKFMS = getMS();
	 publishCommand("p keyframe"); // take KF
	 }
	 else if (getMS() - lastForceKFMS >= 1000)
	 {
	 lastForceKFMS = getMS();
	 publishCommand("p keyframe"); // take KF
	 }

	 // good position, update trajectory
	 if (statePtr->ptamState == statePtr->PTAM_GOOD
	 || statePtr->ptamState == statePtr->PTAM_BEST
	 || statePtr->ptamState == statePtr->PTAM_TOOKKF)
	 {
	 while (trajectory.size() >= trajectoryMaxSize)
	 {
	 trajectory.pop_back();
	 }
	 trajectory.push_front(
	 DronePosition(
	 TooN::makeVector(statePtr->x, statePtr->y, statePtr->z),
	 statePtr->yaw));
	 }

	 // as long as no KI present:
	 // pop next KI (if next KI present).
	 while (currentKI == NULL && !commandQueue.empty())
	 {
	 // lost -> INTERRUPT!
	 if (statePtr->ptamState == statePtr->PTAM_LOST)
	 {
	 currentKIString = "Recovering";
	 // Recovering
	 if (!trajectory.empty())
	 {
	 delete currentKI;
	 currentKI = new KIRecover(trajectory.front(), 0.1);
	 currentKI->setPointers(this, &controller);
	 }
	 else
	 {
	 sendControlToDrone(hoverCommand);
	 ROS_WARN(
	 "lost track, and there is no previous tracked position to try -> sending HOVER");
	 }
	 }
	 else
	 {
	 popNextCommand(statePtr);
	 }
	 }

	 if (currentKI != NULL)
	 {
	 // lost -> INTERRUPT!
	 if (statePtr->ptamState == statePtr->PTAM_LOST
	 && currentKIString != "Recovering")
	 {
	 commandQueue.push_front(currentKIString);
	 currentKIString = "Recovering";
	 // Recovering
	 if (!trajectory.empty())
	 {
	 delete currentKI;
	 currentKI = new KIRecover(trajectory.front(), 0.1);
	 currentKI->setPointers(this, &controller);
	 ROS_WARN("lost track -> start RECOVERING");
	 }
	 else
	 {
	 sendControlToDrone(hoverCommand);
	 ROS_WARN(
	 "lost track, and there is no previous tracked position to try -> sending HOVER");
	 }
	 }
	 // let current KI control.
	 else if (currentKI->update(statePtr))
	 {
	 // RECOVERING
	 if (currentKIString == "Recovering")
	 {
	 if (statePtr->ptamState == statePtr->PTAM_LOST)
	 {
	 if (!trajectory.empty())
	 {
	 trajectory.pop_front();
	 delete currentKI;
	 currentKI = new KIRecover(trajectory.front(), 0.01);
	 currentKI->setPointers(this, &controller);
	 ROS_WARN(
	 "lost track, recover failed -> start RECOVERING again");
	 }
	 else
	 {
	 sendControlToDrone(hoverCommand);
	 ROS_WARN(
	 "lost track, and there is no previous tracked position to try -> sending HOVER");
	 }
	 }
	 else if (statePtr->ptamState == statePtr->PTAM_GOOD
	 || statePtr->ptamState == statePtr->PTAM_BEST)
	 {
	 publishCommand("p keyframe"); // take KF
	 delete currentKI;
	 currentKI = NULL;
	 currentKIString.clear();
	 ROS_WARN("recovered -> FORCE KF");
	 }
	 else if (statePtr->ptamState == statePtr->PTAM_TOOKKF)
	 {
	 delete currentKI;
	 currentKI = NULL;
	 currentKIString.clear();
	 ROS_WARN("recovered -> NO FORCE KF");
	 }
	 }
	 // there is no other KIs in the queue, but the last KI is done.
	 else
	 {
	 // it it not optimal to send hover, but it is reasonable and easy
	 sendControlToDrone(hoverCommand);
	 ROS_WARN(
	 "Autopilot is Controlling, but there is no more KI -> sending HOVER");
	 }
	 }
	 else
	 {
	 // not lost or recovering
	 }
	 }
	 // if there is no current KI now, we obviously have no current goal -> send drone hover
	 // it it not optimal to send hover, but it is reasonable and easy
	 else if (isControlling)
	 {
	 // lost -> RECOVER
	 if (statePtr->ptamState == statePtr->PTAM_LOST)
	 {
	 currentKIString = "Recovering";
	 // Recovering
	 if (!trajectory.empty())
	 {
	 delete currentKI;
	 currentKI = new KIRecover(trajectory.front(), 0.01);
	 currentKI->setPointers(this, &controller);
	 }
	 else
	 {
	 sendControlToDrone(hoverCommand);
	 ROS_WARN(
	 "lost track, and there is no previous tracked position to try -> sending HOVER");
	 }
	 }
	 else
	 {
	 sendControlToDrone(hoverCommand);
	 ROS_WARN(
	 "Autopilot is Controlling, but KI is NULL -> sending HOVER");
	 }
	 }

	 pthread_mutex_unlock(&commandQueue_CS);
	 */
}

void ControlNode::navdataCb(
		const ardrone_autonomy::NavdataConstPtr navdataPtr) {
	altdMM = navdataPtr->altd;
}

// pops next command(s) from queue (until one is found thats not "done" yet).
// assumes propery of command queue lock exists (!)
void ControlNode::popNextCommand(
		const tum_ardrone::filter_stateConstPtr statePtr) {
	// should actually not happen., but to make sure:
	// delete existing KI.
	if (currentKI != NULL) {
		delete currentKI;
		currentKI = NULL;
	}

	// read next command.
	while (currentKI == NULL && commandQueue.size() > 0) {
		std::string command = commandQueue.front();
		commandQueue.pop_front();
		bool commandUnderstood = false;

		// print me
		ROS_INFO("executing command: %s", command.c_str());

		int p;
		char buf[100];
		float parameters[12];

		// [Devesh]
		char snap_num;

		//int pi;

		// replace macros
		if ((p = command.find("$POSE$")) != std::string::npos) {
			snprintf(buf, 100, "%.3f %.3f %.3f %.3f", statePtr->x, statePtr->y,
					statePtr->z, statePtr->yaw);
			command.replace(p, 6, buf);
		}
		if ((p = command.find("$REFERENCE$")) != std::string::npos) {
			snprintf(buf, 100, "%.3f %.3f %.3f %.3f",
					parameter_referenceZero.pos[0],
					parameter_referenceZero.pos[1],
					parameter_referenceZero.pos[2],
					parameter_referenceZero.yaw);
			command.replace(p, 11, buf);
		}

		// -------- commands -----------
		// autoInit
		if (sscanf(command.c_str(), "autoInit %f %f %f %f", &parameters[0],
				&parameters[1], &parameters[2], &parameters[3]) == 4) {
			currentKI = new KIAutoInit(true, parameters[0], parameters[1],
					parameters[2], parameters[3], true);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
		}

		else if (sscanf(command.c_str(), "autoTakeover %f %f %f %f",
				&parameters[0], &parameters[1], &parameters[2], &parameters[3])
				== 4) {
			currentKI = new KIAutoInit(true, parameters[0], parameters[1],
					parameters[2], parameters[3], false);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
		}

		// takeoff
		else if (command == "takeoff") {
			currentKI = new KIAutoInit(false);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
		}

		// setOffset
		else if (sscanf(command.c_str(), "setReference %f %f %f %f",
				&parameters[0], &parameters[1], &parameters[2], &parameters[3])
				== 4) {
			parameter_referenceZero = DronePosition(
					TooN::makeVector(parameters[0], parameters[1],
							parameters[2]), parameters[3]);
			commandUnderstood = true;
		}

		// setMaxControl
		else if (sscanf(command.c_str(), "setMaxControl %f", &parameters[0])
				== 1) {
			parameter_MaxControl = parameters[0];
			commandUnderstood = true;
		}

		// setInitialReachDist
		else if (sscanf(command.c_str(), "setInitialReachDist %f",
				&parameters[0]) == 1) {
			parameter_InitialReachDist = parameters[0];
			commandUnderstood = true;
		}

		// setStayWithinDist
		else if (sscanf(command.c_str(), "setStayWithinDist %f", &parameters[0])
				== 1) {
			parameter_StayWithinDist = parameters[0];
			commandUnderstood = true;
		}

		// setStayTime
		else if (sscanf(command.c_str(), "setStayTime %f", &parameters[0])
				== 1) {
			parameter_StayTime = parameters[0];
			commandUnderstood = true;
		}

		// setLineSpeed [ziquan]
		else if (sscanf(command.c_str(), "setLineSpeed %f", &parameters[0])
				== 1) {
			parameter_LineSpeed = parameters[0];
			commandUnderstood = true;
		}

		// setSpinSpeed [ziquan]
		else if (sscanf(command.c_str(), "setSpinSpeed %f", &parameters[0])
				== 1) {
			parameter_SpinSpeed = parameters[0];
			commandUnderstood = true;
		}

		// setGSVScaler [ziquan]
		else if (sscanf(command.c_str(), "setGSVScaler %f", &parameters[0])
				== 1) {
			parameter_GSVScalar = parameters[0];
			commandUnderstood = true;
		}

		// goto
		else if (sscanf(command.c_str(), "goto %f %f %f %f", &parameters[0],
				&parameters[1], &parameters[2], &parameters[3]) == 4) {
			currentKI = new KIFlyTo(
					DronePosition(
							TooN::makeVector(parameters[0], parameters[1],
									parameters[2])
									+ parameter_referenceZero.pos,
							parameters[3] + parameter_referenceZero.yaw),
					parameter_StayTime, parameter_MaxControl,
					parameter_InitialReachDist, parameter_StayWithinDist);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}

		// [Devesh]
		// gotoRelDir: When to want to go to a location relative to
		// a co-ordinate system aligned with your yaw angle. (i.e, when
		// straight-ahead for the drone is imagined as y-axis at that instant)
		else if (sscanf(command.c_str(), "gotoRelDir %f %f %f %f",
				&parameters[0], &parameters[1], &parameters[2], &parameters[3])
				== 4) {
			currentKI = new KIFlyToRelDir(
					DronePosition(
							// acos(yaw) - bsin(yaw), asin(yaw) + bcos(yaw)
							// statePtr->yaw is clockwise. So, I take negative of it.
							TooN::makeVector(
									parameters[0]
											* cos(-statePtr->yaw * M_PI / 180)
											- parameters[1]
													* sin(
															-statePtr->yaw
																	* M_PI
																	/ 180),
									parameters[0]
											* sin(-statePtr->yaw * M_PI / 180)
											+ parameters[1]
													* cos(
															-statePtr->yaw
																	* M_PI
																	/ 180),
									parameters[2])
									+ parameter_referenceZero.pos,
							parameters[3] + parameter_referenceZero.yaw),
					parameter_StayTime, parameter_MaxControl,
					parameter_InitialReachDist, parameter_StayWithinDist);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;

			//std::cout << "checkpoint thingys are " << parameters[0]*cos(-statePtr->yaw * PI / 180) - parameters[1]*sin(-statePtr->yaw * PI / 180)
			//    << parameters[0]*sin(-statePtr->yaw * PI / 180) + parameters[1]*cos(-statePtr->yaw * PI / 180) << statePtr->yaw << endl;

		}

		// [Devesh]
//        else if (sscanf(command.c_str(), "goAlongRelDir %f %f %f %f",
//                &parameters[0], &parameters[1], &parameters[2], &parameters[3])
//                == 4)
//        {
//            currentKI = new KIFlyAlong(
//            // current position
//                    DronePosition(
//                            TooN::makeVector(statePtr->x, statePtr->y,
//                                    statePtr->z), statePtr->yaw),
//                    // direction
//                    TooN::makeVector(
//                            parameters[0] * cos(-statePtr->yaw * PI / 180)
//                                    - parameters[1]
//                                            * sin(-statePtr->yaw * PI / 180),
//                            parameters[0] * sin(-statePtr->yaw * PI / 180)
//                                    + parameters[1]
//                                            * cos(-statePtr->yaw * PI / 180),
//                            parameters[2]),
//                    // line speed
//                    parameter_LineSpeed,
//                    // distance
//                    sqrt(
//                            parameters[0] * parameters[0]
//                                    + parameters[1] * parameters[1]
//                                    + parameters[2] * parameters[2]));
//            currentKI->setPointers(this, &controller);
//            commandUnderstood = true;
//        }

		// moveBy
		else if (sscanf(command.c_str(), "moveBy %f %f %f %f", &parameters[0],
				&parameters[1], &parameters[2], &parameters[3]) == 4) {
			currentKI = new KIFlyTo(
					DronePosition(
							TooN::makeVector(parameters[0], parameters[1],
									parameters[2])
									+ controller.getCurrentTarget().pos,
							parameters[3] + controller.getCurrentTarget().yaw),
					parameter_StayTime, parameter_MaxControl,
					parameter_InitialReachDist, parameter_StayWithinDist);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}

		// moveByRel
		else if (sscanf(command.c_str(), "moveByRel %f %f %f %f",
				&parameters[0], &parameters[1], &parameters[2], &parameters[3])
				== 4) {
			currentKI = new KIFlyTo(
					DronePosition(
							TooN::makeVector(parameters[0] + statePtr->x,
									parameters[1] + statePtr->y,
									parameters[2] + statePtr->z),
							parameters[3] + statePtr->yaw), parameter_StayTime,
					parameter_MaxControl, parameter_InitialReachDist,
					parameter_StayWithinDist);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}

		// land
		else if (command == "land") {
			currentKI = new KILand();
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}

		// setScaleFP
		else if (command == "lockScaleFP") {
			publishCommand("p lockScaleFP");
			commandUnderstood = true;
		}

		else if (sscanf(command.c_str(), "snap %c", &snap_num) == 1) {
			std_msgs::String s;
			s.data = "Snap ";
			s.data += snap_num;
			interface_directions_pub.publish(s);
			commandUnderstood = true;
		}

		// goAlong [ziquan]
//        else if (sscanf(command.c_str(), "goAlong %f %f %f %f", &parameters[0],
//                &parameters[1], &parameters[2], &parameters[3]) == 4)
//        {
//            currentKI = new KIFlyAlong(
//            // current position
//                    DronePosition(
//                            TooN::makeVector(statePtr->x, statePtr->y,
//                                    statePtr->z), statePtr->yaw),
//                    // direction
//                    TooN::makeVector(parameters[0], parameters[1],
//                            parameters[2]),
//                    // line speed
//                    parameter_LineSpeed,
//                    // distance
//                    parameters[3]);
//            currentKI->setPointers(this, &controller);
//            commandUnderstood = true;
//            currentKIString = command;
//        }

		// spin [ziquan]
		else if (sscanf(command.c_str(), "spin %f", &parameters[0]) == 1) {
			currentKI = new KISpin(
			// current position
					DronePosition(
							TooN::makeVector(statePtr->x, statePtr->y,
									statePtr->z), statePtr->yaw),
					// line speed
					parameter_SpinSpeed,
					// distance
					parameters[0]);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}
		// flyAround [ziquan]
		else if (sscanf(command.c_str(), "goAroundR %f %f %f %f %f %f",
				&parameters[0], &parameters[1], &parameters[2], &parameters[3],
				&parameters[4], &parameters[5]) == 6) {
			currentKI = new KIFlyAround(
					TooN::makeVector(parameters[0], parameters[1],
							parameters[2]), parameters[3], statePtr->yaw,
					parameters[4], TooN::makeVector(0.0, 0.0, 1.0),
					parameters[5]);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}

		else if (sscanf(command.c_str(), "goAroundL %f %f %f %f %f %f",
				&parameters[0], &parameters[1], &parameters[2], &parameters[3],
				&parameters[4], &parameters[5]) == 6) {
			currentKI = new KIFlyAround(
					TooN::makeVector(parameters[0], parameters[1],
							parameters[2]), parameters[3], statePtr->yaw,
					parameters[4], TooN::makeVector(0.0, 0.0, -1.0),
					parameters[5]);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}

		// [Devesh]
		// goAroundRepeat direction rotation_angle
		else if (sscanf(command.c_str(), "goAroundRepeat %f %f", &parameters[0],
				&parameters[1]) == 2) {
			currentKI = new KIFlyAround(stored_center, stored_radius,
					statePtr->yaw, parameters[1],
					TooN::makeVector(0.0, 0.0, parameters[0]), stored_speed);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}

		// flyAround [Devesh]
		else if (sscanf(command.c_str(), "goAroundRRelDir %f %f %f %f %f %f",
				&parameters[0], &parameters[1], &parameters[2], &parameters[3],
				&parameters[4], &parameters[5]) == 6) {
			// acos(yaw) - bsin(yaw), asin(yaw) + bcos(yaw)
			// statePtr->yaw is clockwise. So, I take negative of it.
			TooN::Vector<3> centerPoint = TooN::makeVector(
					parameters[0] * cos(-statePtr->yaw * M_PI / 180)
							- parameters[1] * sin(-statePtr->yaw * M_PI / 180),
					parameters[0] * sin(-statePtr->yaw * M_PI / 180)
							+ parameters[1] * cos(-statePtr->yaw * M_PI / 180),
					parameters[2]) + parameter_referenceZero.pos;

			stored_center = centerPoint;
			stored_radius = parameters[3];
			stored_speed = parameters[5];

			currentKI = new KIFlyAround(centerPoint, parameters[3],
					statePtr->yaw, parameters[4],
					TooN::makeVector(0.0, 0.0, 1.0), parameters[5]);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}

		else if (sscanf(command.c_str(), "goAroundLRelDir %f %f %f %f %f %f",
				&parameters[0], &parameters[1], &parameters[2], &parameters[3],
				&parameters[4], &parameters[5]) == 6) {
			// acos(yaw) - bsin(yaw), asin(yaw) + bcos(yaw)
			// statePtr->yaw is clockwise. So, I take negative of it.
			TooN::Vector<3> centerPoint = TooN::makeVector(
					parameters[0] * cos(-statePtr->yaw * M_PI / 180)
							- parameters[1] * sin(-statePtr->yaw * M_PI / 180),
					parameters[0] * sin(-statePtr->yaw * M_PI / 180)
							+ parameters[1] * cos(-statePtr->yaw * M_PI / 180),
					parameters[2]) + parameter_referenceZero.pos;

			stored_center = centerPoint;
			stored_radius = parameters[3];
			stored_speed = parameters[5];

			currentKI = new KIFlyAround(centerPoint, parameters[3],
					statePtr->yaw, parameters[4],
					TooN::makeVector(0.0, 0.0, -1.0), parameters[5]);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}

//        // circleL4 [ziquan]
//        else if (sscanf(command.c_str(), "circleL %f %f %f %f", &parameters[0],
//                &parameters[1], &parameters[2], &parameters[3]) == 4)
//        {
//            TooN::Vector<3> centerPoint = TooN::makeVector(parameters[0],
//                    parameters[1], parameters[2]);
//            TooN::Vector<3> upVector = TooN::makeVector(0.0, 0.0, 1.0);
//            double radius = parameters[3];
//            currentKI = new KIFlyAround(centerPoint, upVector, radius,
//                    parameter_LineSpeed, parameter_StayTime);
//            currentKI->setPointers(this, &controller);
//            commandUnderstood = true;
//            currentKIString = command;
//        }
//        // circleL3 [ziquan]
//        else if (sscanf(command.c_str(), "circleL %f %f %f", &parameters[0],
//                &parameters[1], &parameters[2]) == 3)
//        {
//            TooN::Vector<3> centerPoint = TooN::makeVector(parameters[0],
//                    parameters[1], parameters[2]) + parameter_referenceZero.pos;
//            TooN::Vector<3> currentPoint = TooN::makeVector(statePtr->x,
//                    statePtr->y, statePtr->z);
//            TooN::Vector<3> upVector = TooN::makeVector(0.0, 0.0, 1.0);
//            double radius = sqrt(
//                    (centerPoint - currentPoint)
//                            * (centerPoint - currentPoint));
//            currentKI = new KIFlyAround(centerPoint, upVector, radius,
//                    parameter_LineSpeed, parameter_StayTime);
//            currentKI->setPointers(this, &controller);
//            commandUnderstood = true;
//            snprintf(buf, 100, "circleL %.3f %.3f %.3f %.3f", centerPoint[0],
//                    centerPoint[1], centerPoint[2], radius);
//            currentKIString = std::string(buf);
//        }
//        // circleR4 [ziquan]
//        else if (sscanf(command.c_str(), "circleR %f %f %f %f", &parameters[0],
//                &parameters[1], &parameters[2], &parameters[3]) == 4)
//        {
//            TooN::Vector<3> centerPoint = TooN::makeVector(parameters[0],
//                    parameters[1], parameters[2]);
//            TooN::Vector<3> upVector = TooN::makeVector(0.0, 0.0, -1.0);
//            double radius = parameters[3];
//            currentKI = new KIFlyAround(centerPoint, upVector, radius,
//                    parameter_LineSpeed, parameter_StayTime);
//            currentKI->setPointers(this, &controller);
//            commandUnderstood = true;
//            currentKIString = command;
//        }
//        // circleR3 [ziquan]
//        else if (sscanf(command.c_str(), "circleR %f %f %f", &parameters[0],
//                &parameters[1], &parameters[2]) == 3)
//        {
//            TooN::Vector<3> centerPoint = TooN::makeVector(parameters[0],
//                    parameters[1], parameters[2]) + parameter_referenceZero.pos;
//            TooN::Vector<3> currentPoint = TooN::makeVector(statePtr->x,
//                    statePtr->y, statePtr->z);
//            TooN::Vector<3> upVector = TooN::makeVector(0.0, 0.0, -1.0);
//            double radius = sqrt(
//                    (centerPoint - currentPoint)
//                            * (centerPoint - currentPoint));
//            currentKI = new KIFlyAround(centerPoint, upVector, radius,
//                    parameter_LineSpeed, parameter_StayTime);
//            currentKI->setPointers(this, &controller);
//            commandUnderstood = true;
//            snprintf(buf, 100, "circleR %.3f %.3f %.3f %.3f", centerPoint[0],
//                    centerPoint[1], centerPoint[2], radius);
//            currentKIString = std::string(buf);
//        }
//        else if (sscanf(command.c_str(), "circleRRelDir %f %f %f", &parameters[0],
//                &parameters[1], &parameters[2]) == 3)
//        {
//                            // acos(yaw) - bsin(yaw), asin(yaw) + bcos(yaw)
//                            // statePtr->yaw is clockwise. So, I take negative of it.
//            TooN::Vector<3> centerPoint = TooN::makeVector(
//                        parameters[0]
//                                * cos(-statePtr->yaw * M_PI / 180)
//                                - parameters[1]
//                                        * sin(
//                                                -statePtr->yaw * M_PI
//                                                        / 180),
//                        parameters[0]
//                                * sin(-statePtr->yaw * M_PI / 180)
//                                + parameters[1]
//                                        * cos(
//                                                -statePtr->yaw * M_PI
//                                                        / 180),
//                        parameters[2]) + parameter_referenceZero.pos;
//
//            TooN::Vector<3> currentPoint = TooN::makeVector(statePtr->x,
//                    statePtr->y, statePtr->z);
//            TooN::Vector<3> upVector = TooN::makeVector(0.0, 0.0, -1.0);
//            double radius = sqrt(
//                    (centerPoint - currentPoint)
//                            * (centerPoint - currentPoint));
//            currentKI = new KIFlyAround(centerPoint, upVector, radius,
//                    parameter_LineSpeed, parameter_StayTime);
//            currentKI->setPointers(this, &controller);
//            commandUnderstood = true;
//            snprintf(buf, 100, "circleR %.3f %.3f %.3f %.3f", centerPoint[0],
//                    centerPoint[1], centerPoint[2], radius);
//            currentKIString = std::string(buf);
//        }
		// Qlearning [ziquan]
		else if (sscanf(command.c_str(), "qlearn %f %f", &parameters[0],
				&parameters[1]) == 2) {
			currentKI = new KIQLearning(parameters[0], parameters[1]);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
		}
		// Model-based learning [ziquan]
		else if (sscanf(command.c_str(), "mlearn %f %f", &parameters[0],
				&parameters[1]) == 2) {
			currentKI = new KIModel((int) (parameters[0]), parameters[1]);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
		}
		// Reps [ziquan]
		else if (sscanf(command.c_str(), "reps %f %f", &parameters[0],
				&parameters[1]) == 2) {
			currentKI = new KIRepsExe(parameters[0], parameters[1]);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
		}
		// GSV_goto [ziquan]
		else if (sscanf(command.c_str(), "GSV_goto %f %f", &parameters[0],
				&parameters[1]) == 2) {
			double direction = statePtr->yaw + parameters[0]; // yaw + angle_x
			direction = angleToValidYaw(direction);
			double distance = altdMM * 0.001
					* tan(M_PI_2 - parameters[1] * M_PI / 180); // height * cot(angle_y)
			distance *= parameter_GSVScalar;
			currentKI = new KIFlyTo(
					DronePosition(
							TooN::makeVector(
									statePtr->x
											+ distance
													* sin(
															direction * M_PI
																	/ 180),
									statePtr->y
											+ distance
													* cos(
															direction * M_PI
																	/ 180),
									statePtr->z), direction),
					parameter_StayTime, parameter_MaxControl,
					parameter_InitialReachDist, parameter_StayWithinDist);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}
//// GSV_circle [ziquan]
//		else if (sscanf(command.c_str(), "GSV_circle %f %f", &parameters[0],
//				&parameters[1]) == 2) {
//			double direction = statePtr->yaw + parameters[0];	// yaw + angle_x
//			while (direction < -180) {
//				direction += 360;
//			}
//			while (direction >= 180) {
//				direction -= 360;
//			}
//			double distance = altdMM * 0.001
//					* tan(M_PI_2 - parameters[1] * M_PI / 180);	// height * cot(angle_y)
//			distance *= parameter_GSVScalar;
//			currentKI = new KICircle(
//					// current position
//					DronePosition(
//							TooN::makeVector(statePtr->x, statePtr->y,
//									statePtr->z), statePtr->yaw),
//					// center
//					TooN::makeVector(
//							statePtr->x
//									+ distance * sin(direction * M_PI / 180),
//							statePtr->y
//									+ distance * cos(direction * M_PI / 180),
//							statePtr->z),
//					// upVector
//					TooN::makeVector(0.0, 0.0, -1.0), parameter_LineSpeed,
//					parameter_StayTime);
//			currentKI->setPointers(this, &controller);
//			commandUnderstood = true;
//			currentKIString = command;
//		}
// hover [ziquan]
		else if (command == "hoverlog") {
			currentKI = new KIFlyTo(
					DronePosition(
							TooN::makeVector(statePtr->x, statePtr->y,
									statePtr->z), statePtr->yaw),
					parameter_StayTime, parameter_MaxControl,
					parameter_InitialReachDist, parameter_StayWithinDist, true);
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
			currentKIString = command;
		}

		else if (sscanf(command.c_str(),
				"zigzagboard %f %f %f %f %f %f %f %f %f %f %f %f",
				&parameters[0], &parameters[1], &parameters[2], &parameters[3],
				&parameters[4], &parameters[5], &parameters[9], &parameters[10],
				&parameters[11], &parameters[6], &parameters[7], &parameters[8])
				== 12) {
			// acos(yaw) - bsin(yaw), asin(yaw) + bcos(yaw)
			// statePtr->yaw is clockwise. So, I take negative of it.
			currentKI = new KIZigZagBoard(
					TooN::makeVector(
							parameters[0] * cos(-statePtr->yaw * M_PI / 180)
									- parameters[1]
											* sin(-statePtr->yaw * M_PI / 180),
							parameters[0] * sin(-statePtr->yaw * M_PI / 180)
									+ parameters[1]
											* cos(-statePtr->yaw * M_PI / 180),
							parameters[2]) + parameter_referenceZero.pos,

					TooN::makeVector(
							parameters[3] * cos(-statePtr->yaw * M_PI / 180)
									- parameters[4]
											* sin(-statePtr->yaw * M_PI / 180),
							parameters[3] * sin(-statePtr->yaw * M_PI / 180)
									+ parameters[4]
											* cos(-statePtr->yaw * M_PI / 180),
							parameters[5]) + parameter_referenceZero.pos,

					TooN::makeVector(
							parameters[6] * cos(-statePtr->yaw * M_PI / 180)
									- parameters[7]
											* sin(-statePtr->yaw * M_PI / 180),
							parameters[6] * sin(-statePtr->yaw * M_PI / 180)
									+ parameters[7]
											* cos(-statePtr->yaw * M_PI / 180),
							parameters[8]) + parameter_referenceZero.pos,
					TooN::makeVector(
							parameters[9] * cos(-statePtr->yaw * M_PI / 180)
									- parameters[10]
											* sin(-statePtr->yaw * M_PI / 180),
							parameters[9] * sin(-statePtr->yaw * M_PI / 180)
									+ parameters[10]
											* cos(-statePtr->yaw * M_PI / 180),
							parameters[11]) + parameter_referenceZero.pos, 2,
					92.0 * 16 / sqrt(16 * 16 + 9 * 9),
					92.0 * 9 / sqrt(16 * 16 + 9 * 9));
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
		}
		// [ziquan]
		else if (sscanf(command.c_str(), "zigzagtower %f %f %f %f %f %f %f",
				&parameters[0], &parameters[1], &parameters[2], &parameters[3],
				&parameters[4], &parameters[5], &parameters[6]) == 7) {
			// acos(yaw) - bsin(yaw), asin(yaw) + bcos(yaw)
			// statePtr->yaw is clockwise. So, I take negative of it.
			currentKI = new KIZigZagTower(
					TooN::makeVector(
							parameters[0] * cos(-statePtr->yaw * M_PI / 180)
									- parameters[1]
											* sin(-statePtr->yaw * M_PI / 180),
							parameters[0] * sin(-statePtr->yaw * M_PI / 180)
									+ parameters[1]
											* cos(-statePtr->yaw * M_PI / 180),
							parameters[2]) + parameter_referenceZero.pos,

					TooN::makeVector(
							parameters[3] * cos(-statePtr->yaw * M_PI / 180)
									- parameters[4]
											* sin(-statePtr->yaw * M_PI / 180),
							parameters[3] * sin(-statePtr->yaw * M_PI / 180)
									+ parameters[4]
											* cos(-statePtr->yaw * M_PI / 180),
							parameters[5]) + parameter_referenceZero.pos,
					statePtr->yaw + parameters[6]/2 + parameter_referenceZero.yaw, parameters[6], 2,
					92.0 * 16 / sqrt(16 * 16 + 9 * 9),
					92.0 * 9 / sqrt(16 * 16 + 9 * 9));
			currentKI->setPointers(this, &controller);
			commandUnderstood = true;
		}
		if (!commandUnderstood)
			ROS_INFO("unknown command, skipping!");
	}
}

void ControlNode::comCb(const std_msgs::StringConstPtr str) {
	// only handle commands with prefix c
	if (str->data.length() > 2 && str->data.substr(0, 2) == "c ") {
		std::string cmd = str->data.substr(2, str->data.length() - 2);

		if (cmd.length() == 4 && cmd.substr(0, 4) == "stop") {
			isControlling = false;
			publishCommand("u l Autopilot: Stop Controlling");
			ROS_INFO("STOP CONTROLLING!");
		} else if (cmd.length() == 5 && cmd.substr(0, 5) == "start") {
			isControlling = true;
			publishCommand("u l Autopilot: Start Controlling");
			ROS_INFO("START CONTROLLING!");
		} else if (cmd.length() == 13 && cmd.substr(0, 13) == "clearCommands") {
			pthread_mutex_lock(&commandQueue_CS);
			commandQueue.clear(); // clear command queue.
			controller.clearTarget(); // clear current controller target
			if (currentKI != NULL)
				delete currentKI; // destroy & delete KI.
			currentKI = NULL;
			pthread_mutex_unlock(&commandQueue_CS);

			publishCommand("u l Autopilot: Cleared Command Queue");
			ROS_INFO("Cleared Command Queue!");
		} else {
			pthread_mutex_lock(&commandQueue_CS);
			commandQueue.push_back(cmd);
			pthread_mutex_unlock(&commandQueue_CS);
		}
	}

	// global command: toggle log
	if (str->data.length() == 9 && str->data.substr(0, 9) == "toggleLog") {
		this->toogleLogging();
	}
}

void ControlNode::Loop() {
	ros::Time last = ros::Time::now();
	ros::Time lastStateUpdate = ros::Time::now();

	while (nh_.ok()) {

		// -------------- 1. spin for 50ms, do main controlling part here. ---------------
		while ((ros::Time::now() - last)
				< ros::Duration(minPublishFreq / 1000.0))
			ros::getGlobalCallbackQueue()->callAvailable(
					ros::WallDuration(
							minPublishFreq / 1000.0
									- (ros::Time::now() - last).toSec()));
		last = ros::Time::now();

		// -------------- 2. send hover (maybe). ---------------
		if (isControlling
				&& getMS(ros::Time::now()) - lastControlSentMS
						> minPublishFreq) {
			sendControlToDrone(hoverCommand);
			ROS_WARN(
					"Autopilot enabled, but no estimated pose received - sending HOVER.");
		}

		// -------------- 3. update info. ---------------
		if ((ros::Time::now() - lastStateUpdate) > ros::Duration(0.4)) {
			reSendInfo();
			lastStateUpdate = ros::Time::now();
		}
	}
}
void ControlNode::dynConfCb(tum_ardrone::AutopilotParamsConfig &config,
		uint32_t level) {
	controller.Ki_gaz = config.Ki_gaz;
	controller.Kd_gaz = config.Kd_gaz;
	controller.Kp_gaz = config.Kp_gaz;

	controller.Ki_rp = config.Ki_rp;
	controller.Kd_rp = config.Kd_rp;
	controller.Kp_rp = config.Kp_rp;

	controller.Ki_yaw = config.Ki_yaw;
	controller.Kd_yaw = config.Kd_yaw;
	controller.Kp_yaw = config.Kp_yaw;

	controller.max_gaz_drop = config.max_gaz_drop;
	controller.max_gaz_rise = config.max_gaz_rise;
	controller.max_rp = config.max_rp;
	controller.max_yaw = config.max_yaw;
	controller.agressiveness = config.agressiveness;
	controller.rise_fac = config.rise_fac;
}

pthread_mutex_t ControlNode::tum_ardrone_CS = PTHREAD_MUTEX_INITIALIZER;
void ControlNode::publishCommand(std::string c) {
	std_msgs::String s;
	s.data = c.c_str();
	pthread_mutex_lock(&tum_ardrone_CS);
	tum_ardrone_pub.publish(s);
	pthread_mutex_unlock(&tum_ardrone_CS);
}

void ControlNode::toogleLogging() {
	// logging has yet to be integrated.
}

void ControlNode::sendControlToDrone(ControlCommand cmd) {
	geometry_msgs::Twist cmdT;
	cmdT.angular.z = -cmd.yaw;
	cmdT.linear.z = cmd.gaz;
	cmdT.linear.x = -cmd.pitch;
	cmdT.linear.y = -cmd.roll;

	// assume that while actively controlling, the above for will never be equal to zero, so i will never hover.
	cmdT.angular.x = cmdT.angular.y = 0;
	// cmdT.angular.x = cmdT.angular.x = 0; MODIFIED BY ZIQUAN

	if (isControlling) {
		vel_pub.publish(cmdT);
		lastSentControl = cmd;
	}

	lastControlSentMS = getMS(ros::Time::now());
}

void ControlNode::sendLand() {
	if (isControlling)
		land_pub.publish(std_msgs::Empty());
}
void ControlNode::sendTakeoff() {
	if (isControlling)
		takeoff_pub.publish(std_msgs::Empty());
}
void ControlNode::sendToggleState() {
	if (isControlling)
		toggleState_pub.publish(std_msgs::Empty());
}
void ControlNode::reSendInfo() {
	/*
	 Idle / Controlling (Queue: X)
	 Current:
	 Next:
	 Target: X,X,X,X
	 Error: X,X,X,X
	 */

	DronePosition p = controller.getCurrentTarget();
	TooN::Vector<4> e = controller.getLastErr();
	double ea = sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
	snprintf(buf, 500,
			"u c %s (Queue: %d)\nCurrent: %s\nNext: %s\nTarget: (%.2f,  %.2f,  %.2f), %.1f\nError: (%.2f,  %.2f,  %.2f), %.1f (|.| %.2f)\nCont.: r %.2f, p %.2f, g %.2f, y %.2f",
			isControlling ? "Controlling" : "Idle", (int) commandQueue.size(),
			currentKI == NULL ? "NULL" : currentKI->command.c_str(),
			commandQueue.size() > 0 ? commandQueue.front().c_str() : "NULL",
			p.pos[0], p.pos[1], p.pos[2], p.yaw, e[0], e[1], e[2], e[3], ea,
			lastSentControl.roll, lastSentControl.pitch, lastSentControl.gaz,
			lastSentControl.yaw);

	publishCommand(buf);
}

