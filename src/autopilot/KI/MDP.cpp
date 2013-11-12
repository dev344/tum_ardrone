#include "MDP.h"
#include <cstdlib>

MDP::MDP(int target, double param, double acceleration, int maxPitch)
{
	targetPitch = target;
	unknownParam = param;
	unknownAcceleration = acceleration;
	unknownMaxPitch = maxPitch;
	for (int i = 0; i < 101; i++)
		for (int j = 0; j < 61; j++)
			for (int k = 0; k < 101; k++)
				S[i][j][k] = MDPState(i - 50, j - 30, k - 50);

	// normal distributions
	normalParamFileName = "normalParam.txt";
	infile.open(normalParamFileName.c_str());
	if (infile.is_open())
	{
		for (int i = 0; i < 20; i++)
		{
			infile >> normalParamAscend[i][0] >> normalParamAscend[i][1];
			//distAscend[i] = std::normal_distribution<double> dist(normalParamAscend[i][0], normalParamAscend[i][1]);
		}
		for (int i = 0; i < 20; i++)
		{
			infile >> normalParamDescend[i][0] >> normalParamDescend[i][1];
			//distDescend[i] = std::normal_distribution<double> dist(normalParamDescend[i][0], normalParamDescend[i][1]);
		}
	}
	else
	{
		printf("cannot find param file: normalParam.txt\n");
	}
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
				return -std::min(unknownMaxPitch, old_pitch);
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
				return std::min(unknownMaxPitch, -old_pitch);
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
		if (old_pitch == 0)
		{
			if (a > 0)
			{
				return solveForStepSizeAscend(0, unknownMaxPitch) * unknownParam;
			}
			else
			{
				return -solveForStepSizeAscend(0, unknownMaxPitch) * unknownParam;
			}
		}
		else if (old_pitch > 0)
		{
			if (a > 0)
			{
				return old_pitch + solveForStepSizeAscend(old_pitch, unknownMaxPitch) * unknownParam;
			}
			else
			{
				exit(10);
				return -111111;
				// return old_pitch - solveForStepSizeDescend(old_pitch, -new_energy);
			}
		}
		else	// old_pitch < 0
		{
			if (a > 0)
			{
				exit(11);
				return -222222;
				// return old_pitch + solveForStepSizeDescend(-old_pitch, new_energy);
			}
			else
			{
				return old_pitch - solveForStepSizeAscend(-old_pitch, unknownMaxPitch) * unknownParam;
			}
		}
	}
	else if (new_energy > 0)
	{
		if (old_pitch >= 0)	// the equal sign is important here
		{
			if (a > 0)
			{
				return old_pitch + solveForStepSizeAscend(old_pitch, unknownMaxPitch + new_energy) * unknownParam;
			}
			else
			{
				exit(1);
				return -999999;
			}
		}
		else	// old_pitch < 0
		{
			if (a > 0)
			{
				return old_pitch + solveForStepSizeDescend(-old_pitch, new_energy) * unknownParam;
			}
			else
			{
				exit(2);
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
				exit(3);
				return -777777;
			}
			else
			{
				return old_pitch - solveForStepSizeDescend(old_pitch, -new_energy) * unknownParam;
			}
		}
		else	// old_pitch <= 0, the equal sign is important here
		{
			if (a > 0)
			{
				exit(4);
				return -666666;
			}
			else
			{
				return old_pitch - solveForStepSizeAscend(-old_pitch, unknownMaxPitch - new_energy) * unknownParam;
			}
		}
	}
}

double MDP::solveForNewVelocity(int old_velocityX10, int old_pitch, double new_pitch)
{
	return old_velocityX10 * 0.1 - unknownAcceleration * (tan(old_pitch * 3.14159 / 180.0) + tan(new_pitch * 3.14159 / 180.0)) / 2;
}
