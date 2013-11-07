#include "MDP.h"

MDP::MDP(int target, double param, double acceleration, int maxPitch)
{
	targetPitch = target;
	unknownParam = param;
	unknownAcceleration = acceleration;
	unknownMaxPitch = maxPitch;
	for (int i = 0; i < 25; i++)
		for (int j = 0; j < 11; j++)
			for (int k = 0; k < 25; k++)
				S[i][j][k] = MDPState(i * 4 - 50 + 2, j * 6 - 33 + 3, k * 4 - 50 + 2);
}

MDP::~MDP(void)
{

}

// actionIdx is -1 for forward or +1 for backward
std::vector<std::pair<double,MDPState> > MDP::T(MDPState s, int a)
{
	std::vector<std::pair<double,MDPState> > result;
	
	int new_energy = solveForNewEnergy(s.energy, s.pitch, a);
	double new_pitch = solveForNewPitch(s.pitch, new_energy, a);
	double new_velocity = solveForNewVelocity(s.velocityX10, s.pitch, new_pitch);
		
	result.push_back(std::pair<double, MDPState>(1.0, S[pitchToIndex(new_pitch)][velocityToIndex(new_velocity)][energyToIndex(new_energy)]));
	// can add more states by adjust new_energy e.g. *0.9 *1.1
	return result;
}

double MDP::R(MDPState s)
{
	if (isTerminal(s))
		return 100;
	if (isBadTerminal(s))
		return -50;
	return -1;
}

int MDP::solveForNewEnergy(int old_energy, int old_pitch, int a)
{
	if (old_pitch >= 0)
	{
		if (old_energy >= 0)	// equal sign is important here
		{
			if (a > 0)
			{
				return old_energy;
			}
			else
			{
				return -old_pitch;
			}
		}
		else
		{
			if (a > 0)
			{
				return 0;
			}
			else
			{
				return old_energy;
			}		
		}
	}
	else // old_pitch < 0
	{
		if (old_energy > 0)
		{
			if (a > 0)
			{
				return old_energy;
			}
			else
			{
				return 0;
			}
		}
		else	// old_energy <= 0	equal sign is important here
		{
			if (a > 0)
			{
				return -old_pitch;
			}
			else
			{
				return old_energy;
			}
		}
	}
}

double MDP::solveForNewPitch(int old_pitch, int new_energy, int a)
{
	if (new_energy == 0)
	{
		if (old_pitch > 0)
		{
			if (a > 0)
			{
				return old_pitch + solveForStepSize(std::max(0, unknownMaxPitch - old_pitch)) * unknownParam;
			}
			else
			{
				return old_pitch - solveForStepSize(old_pitch) * unknownParam;
			}
		}
		else
		{
			if (a > 0)
			{
				return old_pitch + solveForStepSize(-old_pitch) * unknownParam;
			}
			else
			{
				return old_pitch - solveForStepSize(std::max(0, unknownMaxPitch + old_pitch)) * unknownParam;
			}
		}
	}
	else if (new_energy > 0)
	{
		if (old_pitch > 0)
		{
			if (a > 0)
			{
				return old_pitch + solveForStepSize(std::max(0, unknownMaxPitch + new_energy - old_pitch)) * unknownParam;
			}
			else
			{
				return -999999;
			}
		}
		else
		{
			if (a > 0)
			{
				return old_pitch + solveForStepSize(-old_pitch) * unknownParam;
			}
			else
			{
				return -888888;
			}
		}
	}
	else	// new_energy < 0
	{
		if (old_pitch > 0)
		{
			if (a > 0)
			{
				return -777777;
			}
			else
			{
				return old_pitch - solveForStepSize(old_pitch) * unknownParam;
			}
		}
		else
		{
			if (a > 0)
			{
				return -666666;
			}
			else
			{
				return old_pitch - solveForStepSize(std::max(0, unknownMaxPitch - new_energy + old_pitch)) * unknownParam;
			}
		}
	}
}

double MDP::solveForNewVelocity(int old_velocityX10, int old_pitch, double new_pitch)
{
	return old_velocityX10 * 0.1 - unknownAcceleration * (tan(old_pitch * 3.14159 / 180.0) + tan(new_pitch * 3.14159 / 180.0)) / 2;
}
