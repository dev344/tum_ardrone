#include "KIQLearning.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"


KIQLearning::KIQLearning(double targetPitchP,
		double timePerTrial)
{
	targetPitch = targetPitchP;
	timePerTrialMs = (int)(1000*timePerTrial);

	numOfUpdate = -99999999;
	numOfActionsInOneTrial = 0;
	numOfTrials = -99999999;
	
	startAtClock = -1;
	isCompleted = false;

	previousPitchIdx = previousVelocityIdx = previousActionIdx = -1;
	previousReward = -99999999;

	qlogFileName = "q_log.txt";
	infile.open(qlogFileName.c_str());
	if(infile.is_open())
	{
		infile >> numOfTrials >> numOfUpdate;
		numOfTrials ++;
	}
	else
	{
		printf("cannot find log file: q_log.txt\n");
		numOfTrials = numOfUpdate = 0;
	}
	for(int i = 0; i < 36; i++){
		for(int j = 0; j < 10; j++){
			for(int k = 0; k < 7; k++){
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
	// waiting for action to take effect
	if (numOfUpdate % MAX_NUM_OF_UPDATES_PER_STEP != 0)
	{
		numOfUpdate++;		
		writeLog();
		return false;
	}

	// perceive
	double pitch = statePtr->pitch;
	double velocity = statePtr->dx;
	LearningState newState = LearningState(pitch, velocity);
	double newReward = reward(newState);

	int pitchIdx = pitchToIndex(pitch);
	int velocityIdx = velocityToIndex(velocity);
	
	// check terminal state : [learn]
	if(isTerminal(newState)){
		// increment and reset
		numOfTrials ++;			// increment total number of trials
		startAtClock = -1;		// restart time
		numOfActionsInOneTrial = 0;	// reset counter of actions

		// update reward
		Q[previousPitchIdx][previousVelocityIdx][6] = newReward;
		
		// print
		printf("trial# %d learnt target = %f, pitch = %f, velocity = %f\n", numOfTrials, targetPitch, newState.pitch, newState.velocity);
	}
	
	// check total number of trials
	if((numOfTrials + 1) % MAX_NUM_OF_TRIALS == 0)
	{
		// print and log
		printf("finish %d trials\n", MAX_NUM_OF_TRIALS);		
		writeLog();

		// return
		return true;
	}
	// check over time : [fail to learn]
	else if(startAtClock > 0 && (getMS() - startAtClock) > timePerTrialMs)
	{
		// increment and reset
		numOfTrials ++;			// increment total number of trials
		startAtClock = -1;		// restart time
		numOfActionsInOneTrial = 0;	// reset counter of actions

		// print
		printf("trial# %d restart\n", numOfTrials);
	}

	// update Q
	if(startAtClock > 0)
	{
		Nsa[previousPitchIdx][previousVelocityIdx][previousActionIdx] ++;
		Q[previousPitchIdx][previousVelocityIdx][previousActionIdx] += alpha(Nsa[previousPitchIdx][previousVelocityIdx][previousActionIdx]) *(previousReward + 0.9 * maxQ(newState) - Q[previousPitchIdx][previousVelocityIdx][previousActionIdx]);
		
	}else{
		startAtClock = getMS();
	}

	// update previous
	previousPitchIdx = pitchIdx;
	previousVelocityIdx = velocityIdx;
	previousActionIdx = actionIdxWithMaxF(newState);
	previousReward = newReward;
	//printf("PitchIdx: %d, VelocityIdx: %d, Action: %d, Qsa: %f, Nsa: %d, steps: %d\n", previousPitchIdx, previousVelocityIdx, previousActionIdx, Q[previousPitchIdx][previousVelocityIdx][previousActionIdx], Nsa[previousPitchIdx][previousVelocityIdx][previousActionIdx], numOfActionsInOneTrial);

	// take action!
	numOfActionsInOneTrial ++;
	node->sendControlToDrone(indexToControlCommand(previousActionIdx));
	return false;	// not done yet (!)
}

void KIQLearning::writeLog()
{
	std::ofstream outfile (qlogFileName.c_str());
	if(outfile.is_open())
	{
		outfile << numOfTrials << "\t" << numOfUpdate << "\n";
		for(int i = 0; i < 36; i++){
			for(int j = 0; j < 10; j++){
				for(int k = 0; k < 7; k++){
					outfile << Q[i][j][k] << "\t" << Nsa[i][j][k] << "\n";
				}
			}	
		}
		outfile.close();
	}
	else
	{
		printf("open failed!\n");
	}
}
