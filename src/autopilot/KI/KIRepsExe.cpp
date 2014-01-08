#include "KIRepsExe.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KIRepsExe::KIRepsExe(double toggleTimeS,
		double timeMaxS)
{
	toggleTimeMs = (int)(1000*toggleTimeS);
	timeMaxMs = (int)(1000*timeMaxS);

	startAtClock = -1;
	isToggled = false;
	isCompleted = false;

	stringstream ssFileName;
	ssFileName << "reps_log_" << toggleTimeMs << ".txt";
	repslogFileName = ssFileName.str();

	char buf[200];
	snprintf(buf,200,"RepsExe, toggleTime %.2f seconds, timeMax %.2f seconds", toggleTimeS, timeMaxS);
	command = buf;
}


KIRepsExe::~KIRepsExe(void)
{
}


bool KIRepsExe::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	if (isCompleted)
	{
		return true;
	}
	// perceive
	double roll = statePtr->roll;
	double pitch = statePtr->pitch;
	//double yaw = statePtr->yaw;
	double dx = statePtr->dx;
	double dy = statePtr->dy;
	double dz = statePtr->dz;
	//double dyaw = statePtr->dyaw;
	
	if (startAtClock < 0)
	{
		if (!(abs(roll) < 2 && abs(pitch) < 2 && //abs(yaw) < 2 &&
			abs(dx) < 0.1 && abs(dy) < 0.1 && abs(dz) < 0.1))
		{
			return false;
		}	
		startAtClock = getMS();
	}
	
	// record down the trajectory
	int timeSinceStart = getMS() - startAtClock;
	trajectory.push_back( make_pair(timeSinceStart, pitch) );

	// check time max
	if(timeSinceStart >= timeMaxMs)
	{
		// print
		printf("RepsExe finish\n");
		isCompleted = true;
		writeLog();
	}
	// toggle
	else if(timeSinceStart >= toggleTimeMs)
	{
		isToggled = true;
	}

	// take action
	node->sendControlToDrone(ControlCommand(0, (isToggled ? -1 : 1), 0, 0));
	return false;	// not done yet (!)
}

void KIRepsExe::writeLog()
{
	std::ofstream outfile (repslogFileName.c_str());
	if(outfile.is_open())
	{
		outfile << toggleTimeMs << "\n";
		for(vector<recordPair>::iterator itr = trajectory.begin();
			itr != trajectory.end(); ++itr)
		{
			outfile << itr->first << "\t" << itr->second << "\n";
		}
		outfile.close();
	}
	else
	{
		printf("create logfile failed!\n");
	}
}
