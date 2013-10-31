#pragma once

#ifndef __KIQLEARNING_H
#define __KIQLEARNING_H

#define MAX_NUM_OF_TRIALS	2
#define MAX_NUM_OF_UPDATES_PER_STEP	5

#include "KIProcedure.h"
#include <iostream>
#include <fstream>
#include <string>

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
	
	int startAtClock;	// the start of a trial
	bool isCompleted;
	
	int previousPitchIdx, previousVelocityIdx, previousActionIdx;
	double previousReward;
	
	double Q[36][10][7]; 	// Pitch x Velocity x Action
	int Nsa[36][10][7];	// Pitch x Velocity x Action	
	
	inline int pitchToIndex(double pitch)
	{
		return std::max(0, std::min(35, (int)(pitch + 90)/5));
	};
	
	inline int velocityToIndex(double velocity)
	{
		return std::max(0, std::min(9, (int)(sqrt(velocity)/0.1)));
	};

	inline ControlCommand indexToControlCommand(int idx)
	{
		if(idx == 0)
			return ControlCommand(0, 0, 0, 0);		//hover
		if(idx == 1)
			return ControlCommand(0, -0.25, 0, 0);		//forward 1/4
		if(idx == 2)
			return ControlCommand(0, -0.5, 0, 0);		//forward 1/2
		if(idx == 3)
			return ControlCommand(0, -0.75, 0, 0);		//forward 3/4
		if(idx == 4)
			return ControlCommand(0, -1, 0, 0);		//forward_max
		if(idx == 5)
			return ControlCommand(0, 1, 0, 0);		//backward_max
		return ControlCommand(0, 0, 0, 0);	//should not be here
	};

	inline bool isTerminal(LearningState s)
	{
		return (abs(s.pitch - targetPitch) < 3 && s.velocity < 0.05);
	};
	
	inline double reward(LearningState s)
	{
		if(isTerminal(s))
			return 100;
		return -1;
	};

	inline double f(double r, int count)
	{
		if(count < 10)
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

		for(int i = 1; i < 6; i++)
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

		int maxIdx = 0;
		double maxF = f(Q[pitchIdx][velocityIdx][0], Nsa[pitchIdx][velocityIdx][0]);

		for(int i = 1; i < 6; i++)
		{
			double tempF = f(Q[pitchIdx][velocityIdx][i], Nsa[pitchIdx][velocityIdx][i]);
			if((tempF > maxF) || ((tempF == maxF) && (rand()%2 == 0)))
			{
				maxIdx = i;
				maxF = tempF;
			}
		}

		return maxIdx;
	};
	void writeLog();
public:
	KIQLearning(double targetPitchP,
		double timePerTrial = 5);

	~KIQLearning(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIQLEARNING_H */
