#pragma once

#ifndef __MDP_H
#define __MDP_H

#include <vector>
#include <utility>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
//#include <random>

struct MDPState{
	inline MDPState() {pitch = velocityX10 = energy = 0;}
	inline MDPState(int pitch, int velocityX10, int energy)
	{
		this->pitch = pitch;
		this->velocityX10 = velocityX10;
		this->energy = energy;
	}
	int pitch, velocityX10, energy;
};

class MDP{
private:
	
	// parameters to be estimate
	double		unknownParam;
	double		unknownAcceleration;
	int			unknownMaxPitch;
	
	// normal distribution
	std::string		normalParamFileName;
	std::ifstream	infile;
	double			normalParamAscend[20][2];
	double			normalParamDescend[20][2];
	//std::normal_distribution<double>	distAscend[20];
	//std::normal_distribution<double>	distDescend[20];
	//std::default_random_engine generator;

	inline bool isTerminal(MDPState s)
	{
		return (s.pitch - targetPitch) <= 3 && (targetPitch - s.pitch) <= 3 && s.pitch * s.velocityX10 >= 0;
	};
	
	inline bool isBadTerminal(MDPState s)
	{
		return false;//s.pitch * targetPitch > 0 && s.pitch * s.velocityX10 < 0 && std::abs(s.pitch) <= unknownMaxPitch;
	};
	
	inline double solveForStepSizeDescend(int thetaCurrent, int thetaTop)
	{
		return normalParamDescend[std::max(0, std::min(19, 20 - (int)(20.0 * thetaCurrent / thetaTop)))][0] * thetaTop / 20.0;
		//return distDescend[std::max(0, std::min(19, 20 - (int)(20.0 * thetaCurrent / thetaTop)))](generator) * thetaTop / 20.0;
	};
	
	inline double solveForStepSizeAscend(int thetaCurrent, int thetaTop)
	{
		return normalParamAscend[std::max(0, std::min(19, (int)(20.0 * thetaCurrent / thetaTop)))][0] * thetaTop / 20.0;
		//return distAscend[std::max(0, std::min(19, (int)(20.0 * thetaCurrent / thetaTop)))](generator) * thetaTop / 20.0;
	};
	
	double	solveForNewPitch(int old_pitch, int new_energy, int a);
	double	solveForNewVelocity(int old_velocityX10, int old_pitch, double new_pitch);
public:
	int		solveForNewEnergy(int old_energy, int old_pitch, int a);

	// target
	int	targetPitch;

	MDP(int target, double param = 1.0, double acceleration = 9.8 * 1.0 / 30.0, int maxPitch = 20);
	~MDP(void);
	
	// standard S, T, R	
	MDPState	S[101][61][101];	// Pitch x Velocity x Energy

	std::vector<std::pair<double,MDPState> >	T(MDPState s, int a);	// a is -1 for forward or +1 for backward
	double		R(MDPState s);

	inline int pitchToIndex(double pitch)
	{
		return std::max(0, std::min(100, (int)round(pitch + 50)));
	};
	
	inline int velocityToIndex(double velocity)
    {
		return std::max(0, std::min(60, (int)round(velocity * 10 + 30)));
	};
	
	inline int energyToIndex(int energy)
	{
		return std::max(0, std::min(100, (energy + 50)));
	};
	
};

#endif /*__MDP_H*/
