#pragma once

#ifndef __KIRECOVER_H
#define __KIRECOVER_H

#include "KIProcedure.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

class KIRecover: public KIProcedure {
private:
	int startAtClock;
	bool targetSet;
//	bool isCompleted;

	int maxTimeMs;

	DronePosition checkpoint;

public:
	KIRecover(DronePosition checkpoint, double maxTimeS = 2);

	~KIRecover(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KIRECOVER_H */
