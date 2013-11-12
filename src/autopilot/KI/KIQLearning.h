#pragma once

#ifndef __KIQLEARNING_H
#define __KIQLEARNING_H

#define MAX_NUM_OF_UPDATES_PER_STEP	5

#include "KIProcedure.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

struct LearningState{
	inline LearningState() {pitch = velocity = 0;}
	inline LearningState(double pitch, double velocity)
	{
		this->pitch = pitch;
		this->velocity = velocity;
	}
	double pitch, velocity;
};

class KIQLearning : public KIProcedure
{
private:
	std::string qlogFileName;
	std::ifstream infile;

	double targetPitch;
	int timePerTrialMs;

	int numOfUpdate;	// waiting for action to take effect
	int numOfActionsInOneTrial;	// num of actions in a trial
	int numOfTrials;	// total num of trials
	int numOfSuccess;	// total num of successful trials
	
	int startAtClock;	// the start of a trial
	bool isCompleted;
	
	int previousPitchIdx, previousVelocityIdx, previousActionIdx;
	double previousReward;
	
	double Q[25][11][3]; 	// Pitch x Velocity x Action
	int Nsa[25][11][3];		// Pitch x Velocity x Action	
	
	inline int pitchToIndex(double pitch)
	{
		return std::max(0, std::min(24, (int)(pitch + 50)/4));
	};
	
	inline int velocityToIndex(double velocity)
	{
		return std::max(0, std::min(10, (int)(velocity + 3.3/0.6)));
	};

	inline ControlCommand indexToControlCommand(int idx)
	{
		if(idx == 0)
			return ControlCommand(0, 1, 0, 0);		//backward_max
		if(idx == 1)
			return ControlCommand(0, -1, 0, 0);		//forward_max
		return ControlCommand(0, 0, 0, 0);			//should not be here
	};

	inline bool isTerminal(LearningState s)
	{
		// the first constrain is clear: the pitch is near to target
		// the second constrain means: 
		// if pitch is positive but velocity is still positive
		// OR if pitch is negative but velocity is still negative
		// Usually, when pitch is positive, velocity should be negative
		return abs(s.pitch - targetPitch) < 2 && s.pitch * s.velocity >= 0;
	};
	
	inline bool isBadTerminal(LearningState s)
	{
		return s.pitch * targetPitch > 0 && s.pitch * s.velocity < 0;
	};

	inline double reward(LearningState s)
	{
		if(isTerminal(s))
			return 100;
		if(isBadTerminal(s))
			return -100;
		return -1;
	};

	inline double f(double r, int count)
	{
		if(count < 3)	// changed from 10 to 3 to increase learning efficiency
			return 100;
		return r;
	};

	inline double alpha(int count)
	{
		return 1.0/count;
	};

	inline double maxQ(LearningState s)
	{
		srand (time(NULL));
		
		int pitchIdx = pitchToIndex(s.pitch);
		int velocityIdx = velocityToIndex(s.velocity);
		double maxQ = Q[pitchIdx][velocityIdx][0];

		for(int i = 1; i < 3; i++)	// find Q value, so include dummy action
		{
			maxQ = std::max(maxQ, Q[pitchIdx][velocityIdx][i]);
		}
	
		return maxQ;
	}

	inline int actionIdxWithMaxF(LearningState s)
	{
		srand (time(NULL));

		int pitchIdx = pitchToIndex(s.pitch);
		int velocityIdx = velocityToIndex(s.velocity);

		double maxF = f(Q[pitchIdx][velocityIdx][0], Nsa[pitchIdx][velocityIdx][0]);
		//double maxQ = Q[pitchIdx][velocityIdx][0];
		std::vector<int> maxIdxCandidates;
		maxIdxCandidates.push_back(0);

		for(int i = 1; i < 2; i++)	// find feasible action, so exclude dummy action
		{
			double tempF = f(Q[pitchIdx][velocityIdx][i], Nsa[pitchIdx][velocityIdx][i]);
			//double tempQ = Q[pitchIdx][velocityIdx][i];
			if(tempF > maxF)
			{
				maxF = tempF;
				maxIdxCandidates.clear();
				maxIdxCandidates.push_back(i);
			}
			else if(tempF == maxF)
			{
				maxIdxCandidates.push_back(i);
			}
		}
		return maxIdxCandidates[rand() % maxIdxCandidates.size()];
	};
	void writeLog();
public:
	KIQLearning(double targetPitchP,
		double timePerTrial = 1);

	~KIQLearning(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIQLEARNING_H */
