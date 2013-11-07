#include "KIModel.h"
//#include "../DroneController.h"
//#include "../ControlNode.h"
//#include "../../HelperFunctions.h"

KIModel::KIModel(int targetPitchP, double timePerTrial)
{
	mlogFileName = "m_log.txt";
	
	mdp = new MDP(targetPitchP, 1.0);

	for (int i = 0; i < 25; i++)
		for (int j = 0; j < 11; j++)
			for (int k = 0; k < 25; k++)
			{
				U[i][j][k] = 0;
				Pi[i][j][k] = 1;
			}
	
	// test use ONLY
	policyIteration();
	writelog();
	// test END
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
		for (int i = 0; i < 25; i++)
			for (int j = 0; j < 11; j++)
				for (int k = 0; k < 25; k++)
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
		for (int i = 0; i < 25; i++)
			for (int j = 0; j < 11; j++)
				for (int k = 0; k < 25; k++)
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
	policyIteration();
	writelog();
	return true;
}

void KIModel::writelog()
{
	std::ofstream outfile (mlogFileName.c_str());
	if(outfile.is_open())
	{
		for(int i = 0; i < 25; i++){
			for(int j = 0; j < 11; j++){
				for(int k = 0; k < 25; k++){
					outfile << mdp->S[i][j][k].pitch << "\t" << mdp->S[i][j][k].velocityX10 << "\t" << mdp->S[i][j][k].energy << "\t" << U[i][j][k] << "\t" << (Pi[i][j][k] > 0 ? "+1" : "-1")  << "\n";
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
