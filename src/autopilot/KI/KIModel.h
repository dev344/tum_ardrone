#pragma once

#ifndef __KIMODEL_H
#define __KIMODEL_H

#include "KIProcedure.h"
#include "MDP.h"
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <utility>

class KIModel : public KIProcedure
{

private:

	std::string mlogFileName;
	std::ifstream infile;

	MDP *mdp;
	double	U[101][61][101];	// Pitch x Velocity x Energy, U[i][j][k] is the utility of S[i][j][k]
	int		Pi[101][61][101];	// Pi[i][j][k] is the action to take in S[i][j][k]
		
	double	expectedUtility(MDPState s, int a);	// use s, a, U, mdp
	void		policyEvaluation();	// use Pi, U and mdp, modifies U
	void		policyIteration();	// use mdp, modifies Pi
		
	inline ControlCommand actionToControlCommand(int a)
	{
		if(a > 0)
			return ControlCommand(0, 1, 0, 0);		//backward_max
		else
			return ControlCommand(0, -1, 0, 0);		//forward_max
	};

	void		writelog();

	int		startAtClock;
	bool		isCompleted;
	int		executionTime;
	int		currentEnergy;

	std::vector<std::pair<MDPState, int> >		recordSA;
	//double	error
public:

	KIModel(int targetPitchP, double executionTimeP);

	~KIModel();
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIMODEL_H */
