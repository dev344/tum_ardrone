#pragma once

#ifndef __MDP_H
#define __MDP_H

#include <vector>
#include <utility>
#include <cmath>

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
	// target
	int			targetPitch;
	
	// parameters to be estimate
	double		unknownParam;
	double		unknownAcceleration;
	int			unknownMaxPitch;
	

	inline bool isTerminal(MDPState s)
	{
		return ((s.pitch - targetPitch) < 3 || (targetPitch - s.pitch) < 3) && s.pitch * s.velocityX10 > 0;
	};
	
	inline bool isBadTerminal(MDPState s)
	{
		return s.pitch * targetPitch > 0 && s.pitch * s.velocityX10 < 0 && std::abs(s.pitch) <= unknownMaxPitch;
	};
	
	// solve int_[0, x](i) = theta/param
	// theta must be positive
	inline double solveForStepSize(int theta)
	{
		return sqrt(theta * 2.0 / unknownParam);
	};
	
	int		solveForNewEnergy(int old_energy, int old_pitch, int a);
	double	solveForNewPitch(int old_pitch, int new_energy, int a);
	double	solveForNewVelocity(int old_velocityX10, int old_pitch, double new_pitch);
	
public:
	MDP(int target, double param, double acceleration = 9.8 * (5.0/30), int maxPitch = 20);
	~MDP(void);
	
	// standard S, T, R	
	MDPState	S[25][11][25];	// Pitch x Velocity x Energy

	std::vector<std::pair<double,MDPState> >	T(MDPState s, int a);	// a is -1 for forward or +1 for backward
	double		R(MDPState s);

	inline int pitchToIndex(double pitch)
	{
		return std::max(0, std::min(24, (int)(pitch + 50)/4));
	};
	
	inline int velocityToIndex(double velocity)
    {
		return std::max(0, std::min(10, (int)((velocity + 3.3)/0.6)));
	};
	
	inline int energyToIndex(int energy)
	{
		return std::max(0, std::min(24, (energy + 50)/4));
	};
	
};

#endif /*__MDP_H*/
