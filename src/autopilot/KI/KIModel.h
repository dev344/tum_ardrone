#pragma once

#ifndef __KIMODEL_H
#define __KIMODEL_H

#include "KIProcedure.h"
#include "MDP.h"
#include <string>
#include <iostream>
#include <fstream>

class KIModel : public KIProcedure
{

private:

	std::string mlogFileName;
	std::ifstream infile;

	MDP *mdp;
	double	U[25][11][25];	// Pitch x Velocity x Energy, U[i][j][k] is the utility of S[i][j][k]
	int		Pi[25][11][25];	// Pi[i][j][k] is the action to take in S[i][j][k]
		
	double	expectedUtility(MDPState s, int a);	// use s, a, U, mdp
	void		policyEvaluation();	// use Pi, U and mdp, modifies U
	void		policyIteration();	// use mdp, modifies Pi
		
	void		writelog();
public:

	KIModel(int targetPitchP, double timePerTrial);

	~KIModel();
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIMODEL_H */
