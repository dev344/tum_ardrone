#pragma once

#ifndef __KISPIN_H
#define __KISPIN_H

#include "KIProcedure.h"

class KISpin : public KIProcedure
{
private:
	bool	spinSet;
	bool	isCompleted;

	DronePosition startPosition;
	double	spinSpeed;
	double	distance;

	double	checkpoint;

public:
	KISpin(DronePosition startPositionP,
		double spinSpeedP = 0.5,
		double distanceP = 180);

	~KISpin(void);
	bool update(const tum_ardrone::filter_stateConstPtr statePtr);
};

#endif /* __KISPIN_H */
