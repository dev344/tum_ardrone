#include "KIModel.h"
#include "../DroneController.h"
#include "../ControlNode.h"
#include "../../HelperFunctions.h"

KIModel::KIModel(int targetPitchP, double executionTimeP)
{
	mlogFileName = "m_log.txt";
	
	mdp = new MDP(targetPitchP, 0.5);
	
	// init policy from file or re-compute
	std::ifstream infile (mlogFileName.c_str());
	if (infile.is_open())
	{
		for (int i = 0; i < 101; i++)
			for (int j = 0; j < 61; j++)
				for (int k = 0; k < 101; k++)
				{
					infile >> U[i][j][k] >> Pi[i][j][k];
				}
	}
	else
	{
		for (int i = 0; i < 101; i++)
			for (int j = 0; j < 61; j++)
				for (int k = 0; k < 101; k++)
				{
					U[i][j][k] = 0;
					Pi[i][j][k] = 1;
				}
		policyIteration();
		writelog();
	}

	startAtClock = -1;
	isCompleted = false;
	executionTime = (int)(1000*executionTimeP);
	currentEnergy = 0;

	char buf[200];
	snprintf(buf,200,"model learning %.2f, executionTime: %.2f seconds",  mdp->targetPitch, executionTimeP);
	command = buf;
}

KIModel::~KIModel()
{
	delete mdp;
}

// use s, a, U, mdp
double KIModel::expectedUtility(MDPState s, int a)
{
	std::vector<std::pair<double,MDPState> > neighbors = mdp->T(s, a);
	double result = 0;
	for (int m = 0; m < neighbors.size(); m++)
	{
		result += neighbors[m].first * U[mdp->pitchToIndex(neighbors[m].second.pitch)]
												[mdp->velocityToIndex(neighbors[m].second.velocityX10 * 0.1)]
												[mdp->energyToIndex(neighbors[m].second.energy)];
	}
	return result;
}

// use Pi, U and mdp, modifies U
void KIModel::policyEvaluation()
{

	int loop = 20;
	while (loop-- > 0)
	{
		for (int i = 0; i < 101; i++)
			for (int j = 0; j < 61; j++)
				for (int k = 0; k < 101; k++)
				{
					U[i][j][k] = mdp->R(mdp->S[i][j][k]) + 0.9 * expectedUtility(mdp->S[i][j][k], Pi[i][j][k]);
				}
	}
}

// use mdp, modifies Pi
void KIModel::policyIteration()
{
	//srand (time(NULL));
	//rand();

	while(true)
	{
		policyEvaluation();
		bool unchanged = true;
		for (int i = 0; i < 101; i++)
			for (int j = 0; j < 61; j++)
				for (int k = 0; k < 101; k++)
				{
					int argmaxAction = (expectedUtility(mdp->S[i][j][k],1) > expectedUtility(mdp->S[i][j][k],-1) ? 1 : -1);
					if (argmaxAction != Pi[i][j][k])
					{
						Pi[i][j][k] = argmaxAction;
						unchanged = false;
					}
				}
		if (unchanged)
			break;
	}
	
}

bool KIModel::update(const tum_ardrone::filter_stateConstPtr statePtr)
{
	// perceive
	double pitch = statePtr->pitch;
	double velocity = statePtr->dy;
	if (startAtClock < 0)
	{
		// start executing policy from 
		// [pitch, velocity, energy] = [~0, ~0, 0]
		if (pitch < 2 && pitch > -2 && velocity < 0.3 && velocity > -0.3)
		{
			startAtClock = getMS();
		}
		else
		{
			return false;
		} 
	}
	// check terminal
	if ((pitch - mdp->targetPitch) < 5 && (mdp->targetPitch - pitch) < 5 && pitch * velocity >= 0)
	{
		// print
		printf("successful execution: target %d, pitch %f, velocity %f\n", mdp->targetPitch, pitch, velocity);

		isCompleted = true;
	}
	else if ((getMS() - startAtClock) > executionTime)
	{
		// print
		printf("falied execution\n");

		isCompleted = true;
	}
	
	if (isCompleted)
	{
		for (int i = 0; i < recordSA.size(); i++)
		{
			std::vector<std::pair<double,MDPState> > neighbors = mdp->T(recordSA[i].first, recordSA[i].second);
			printf("p: %d, vX10: %d, e: %d, a:%d, want to end up in p: %d, vX10: %d, e: %d\n", recordSA[i].first.pitch, recordSA[i].first.velocityX10, recordSA[i].first.energy, recordSA[i].second, neighbors[0].second.pitch, neighbors[0].second.velocityX10, neighbors[0].second.energy);
		}
		return true;
	}
	
	int pitchIdx = mdp->pitchToIndex(pitch);
	int velocityIdx = mdp->velocityToIndex(velocity);
	int energyIdx = mdp->energyToIndex(currentEnergy);	
	int action = Pi[pitchIdx][velocityIdx][energyIdx];
	// record SA pair
	recordSA.push_back(std::pair<MDPState,int>(mdp->S[pitchIdx][velocityIdx][energyIdx], action));
	node->sendControlToDrone(actionToControlCommand(action));
	// update energy	
	currentEnergy = mdp->solveForNewEnergy(currentEnergy, mdp->S[pitchIdx][velocityIdx][energyIdx].pitch, action);
	return false;
}

void KIModel::writelog()
{
	std::ofstream outfile (mlogFileName.c_str());
	if(outfile.is_open())
	{
		for(int i = 0; i < 101; i++){
			for(int j = 0; j < 61; j++){
				for(int k = 0; k < 101; k++){
					outfile << U[i][j][k] << "\t" << Pi[i][j][k] << "\n";
					//std::vector<std::pair<double,MDPState> > neighbors = mdp->T(mdp->S[i][j][k], Pi[i][j][k]);
					//outfile << mdp->S[i][j][k].pitch << "\t" << mdp->S[i][j][k].velocityX10 << "\t" << mdp->S[i][j][k].energy << "\t" << U[i][j][k] << "\t" << Pi[i][j][k]  << "\t->>\t" << neighbors[0].second.pitch << "\t" << neighbors[0].second.velocityX10 << "\t" << neighbors[0].second.energy;
					//std::vector<std::pair<double,MDPState> > neighbors2 = mdp->T(mdp->S[i][j][k], -Pi[i][j][k]);
					//outfile << "\totherwise\t" << neighbors2[0].second.pitch << "\t" << neighbors2[0].second.velocityX10 << "\t" << neighbors2[0].second.energy << "\n";
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
