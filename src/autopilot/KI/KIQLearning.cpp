#include "KIQLearning.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KIQLearning::KIQLearning(double targetPitchP,
		double timePerTrial)
{
	targetPitch = targetPitchP;
	timePerTrialMs = (int)(1000*timePerTrial);

	numOfUpdate = 0;
	numOfActionsInOneTrial = 0;
	numOfTrials = -99999999;
	numOfSuccess = -99999999;

	startAtClock = -1;
	isCompleted = false;

	previousPitchIdx = previousVelocityIdx = previousActionIdx = -1;
	previousReward = -99999999;

	qlogFileName = "q_log.txt";
	infile.open(qlogFileName.c_str());
	if(infile.is_open())
	{
		infile >> numOfTrials >> numOfSuccess;
	}
	else
	{
		printf("cannot find log file: q_log.txt\n");
		numOfTrials = numOfSuccess = 0;
	}
	for(int i = 0; i < 25; i++){
		for(int j = 0; j < 11; j++){
			for(int k = 0; k < 3; k++){
				if(infile.is_open())
				{
					infile >> Q[i][j][k] >> Nsa[i][j][k];
				}
				else
				{
					Q[i][j][k] = Nsa[i][j][k] = 0;
				}
			}
		}	
	}
	if(infile.is_open())
	{
		infile.close();
	}
	
	char buf[200];
	snprintf(buf,200,"Qlearning %.2f, timePerTrial: %.2f seconds", targetPitch, timePerTrial);
	command = buf;
}


KIQLearning::~KIQLearning(void)
{
}


bool KIQLearning::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	
	// perceive
	double pitch = statePtr->pitch;
	double velocity = statePtr->dy;
	
	if (startAtClock < 0)
	{
		if (!(abs(pitch) < 2 && velocity <= 0 && velocity >= -0.3))
		{
			return false;
		}
	}

	LearningState newState = LearningState(pitch, velocity);
	double newReward = reward(newState);

	int pitchIdx = pitchToIndex(pitch);
	int velocityIdx = velocityToIndex(velocity);
	
	// check terminal state : [learnt]
	if(isTerminal(newState))
	{
		// increment and reset
		numOfTrials ++;			// increment total number of trials
		numOfSuccess ++;			// increment total number of successful trials
		//startAtClock = -1;		// restart time
		numOfActionsInOneTrial = 0;	// reset counter of actions
		
		Q[pitchIdx][velocityIdx][2] = newReward;

		// print
		printf("finish %d trials, good terminal target = %f, [pitch = %f, velocity = %f]\n", numOfTrials, targetPitch, newState.pitch, newState.velocity);

		isCompleted = true;
	}
	else if(isBadTerminal(newState))
	{
		// increment and reset
		numOfTrials ++;			// increment total number of trials
		//startAtClock = -1;		// restart time
		numOfActionsInOneTrial = 0;	// reset counter of actions
		
		Q[pitchIdx][velocityIdx][2] = newReward;
		
		// print
		printf("finish %d trials, bad terminal target = %f, [pitch = %f, velocity = %f]\n", numOfTrials, targetPitch, newState.pitch, newState.velocity);

		isCompleted = true;
	}
	// check over time : [fail to learn within time]
	else if(startAtClock > 0 && (getMS() - startAtClock) > timePerTrialMs)
	{
		// increment and reset
		numOfTrials ++;			// increment total number of trials
		//startAtClock = -1;		// restart time
		numOfActionsInOneTrial = 0;	// reset counter of actions

		// print
		printf("finish %d trials, restart\n", numOfTrials);

		isCompleted = true;
	}

	// update Q
	if(startAtClock > 0)
	{
		Nsa[previousPitchIdx][previousVelocityIdx][previousActionIdx] ++;
		Q[previousPitchIdx][previousVelocityIdx][previousActionIdx] += alpha(Nsa[previousPitchIdx][previousVelocityIdx][previousActionIdx]) *(previousReward + 0.9 * maxQ(newState) - Q[previousPitchIdx][previousVelocityIdx][previousActionIdx]);
	}else{
		startAtClock = getMS();
	}
	
	// learnt or run out of time
	if (isCompleted)
	{
		writeLog();
		return true;
	}

	// waiting for action to take effect
	if (numOfUpdate % MAX_NUM_OF_UPDATES_PER_STEP != 0)
	{
		numOfUpdate++;
		return false;
	}

	// update previous
	previousPitchIdx = pitchIdx;
	previousVelocityIdx = velocityIdx;
	previousActionIdx = actionIdxWithMaxF(newState);
	previousReward = newReward;
	
	// take action: [fail to learn but still have time]
	numOfActionsInOneTrial ++;
	node->sendControlToDrone(indexToControlCommand(previousActionIdx));
	return false;	// not done yet (!)
}

void KIQLearning::writeLog()
{
	std::ofstream outfile (qlogFileName.c_str());
	if(outfile.is_open())
	{
		outfile << numOfTrials << "\t" << numOfSuccess << "\n";
		for(int i = 0; i < 25; i++){
			for(int j = 0; j < 11; j++){
				for(int k = 0; k < 3; k++){
					outfile << Q[i][j][k] << "\t" << Nsa[i][j][k] << "\t";
				}
				outfile << "\n";
			}	
		}
		outfile.close();
	}
	else
	{
		printf("open failed!\n");
	}
}
