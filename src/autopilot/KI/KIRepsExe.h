#pragma once

#ifndef __KIREPSEXE_H
#define __KIREPSEXE_H

#include "KIProcedure.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <utility>

using namespace std;

typedef pair<int, double> recordPair;	// pair.first is toggle time in milli-second
					// pair.second is the pitch angle

class KIRepsExe : public KIProcedure
{
private:
	int toggleTimeMs;
	int timeMaxMs;
	
	int startAtClock;	// the start of a trial
	bool isToggled;
	bool isCompleted;

	string repslogFileName;
	//ifstream infile;
	vector<recordPair> trajectory;
	void writeLog();
public:
	KIRepsExe(double toggleTimeS,
		double timeMaxS = 2);

	~KIRepsExe(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIREPSEXE_H */
