#pragma once
 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __HELPERFUNCTIONS_H
#define __HELPERFUNCTIONS_H
 
 
#include <stdlib.h>
#include <TooN/TooN.h>
#include <TooN/so3.h>
#include "ros/ros.h"

/****************************************************
*********** Pose-Representation Conversion **********
****************************************************/
// SEEE http://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel

// the drone coordinate system is:
// x-axis: to the left
// y-axis: up
// z-axis: forward
// roll rhs-system correct
// pitch: rhs-system *-1;
// yaw: rhs system *-1;

inline static TooN::SO3<> rpy2rod(double roll, double pitch, double yaw)
{
	TooN::Matrix<3,3> mat;

	pitch /= 180/M_PI;
	roll /= 180/M_PI;
	yaw /= -180/M_PI;

	double sa = sin(yaw);	// a is yaw = psi
	double ca = cos(yaw);
	double sb = sin(roll);	// b is roll = phi
	double cb = cos(roll);
	double sg = sin(pitch);	// g is pitch = theta
	double cg = cos(pitch);

	mat(0,0) = ca*cb;
	mat(0,1) = sa*cb;
	mat(0,2) = -sb;

	mat(1,0) = ca*sb*sg-sa*cg;
	mat(1,1) = sa*sb*sg+ca*cg;
	mat(1,2) = cb*sg;

	mat(2,0) = ca*sb*cg+sa*sg; 
	mat(2,1) = sa*sb*cg-ca*sg;
	mat(2,2) = cb*cg;

	//mat = mat.T();

	TooN::SO3<> res = mat;
	return res.inverse();
}

inline static void rod2rpy(TooN::SO3<> trans, double* roll, double* pitch, double* yaw)
{
	TooN::Matrix<3,3> mat = trans.inverse().get_matrix();//.T();

	*roll = atan2(-mat(0,2),sqrt(mat(0,0)*mat(0,0) + mat(0,1)*mat(0,1)));
	*yaw = atan2(mat(0,1)/cos(*roll),mat(0,0)/cos(*roll));
	*pitch = atan2(mat(1,2)/cos(*roll),mat(2,2)/cos(*roll));
	
	*pitch *= 180/M_PI;
	*roll *= 180/M_PI;
	*yaw *= -180/M_PI;


	while(*pitch > 180) *pitch -= 360;
	while(*pitch < -180) *pitch += 360;
	while(*roll > 180) *roll -= 360;
	while(*roll < -180) *roll += 360;
	while(*yaw > 180) *yaw -= 360;
	while(*yaw < -180) *yaw += 360;
}



extern unsigned int ros_header_timestamp_base;
// gets a relative ms-time from a ROS time.
// can only be used to compute time differences, as it has no absolute reference.
inline static int getMS(ros::Time stamp = ros::Time::now())
{
	if(ros_header_timestamp_base == 0)
	{
		ros_header_timestamp_base = stamp.sec;
		std::cout << "set ts base to " << ros_header_timestamp_base << std::endl;
	}
	int mss = (stamp.sec - ros_header_timestamp_base) * 1000 + stamp.nsec/1000000;

	if(mss < 0)
		std::cout << "ERROR: negative timestamp..."<< std::endl;
	return mss;
}

// [ziquan]
// (0,1,z) -> yaw = 0
// clockwise -> yaw > 0
// otherwise -> yaw < 0
inline static double vectorToYaw(TooN::Vector<3> v) {
	// base vector = (0,1,0)
	// yaw angle = arccos ( v<2> . (0,1) / |v<2>| * |(0,1)| )
	double result = 180.0 / M_PI
			* acos(v[1] / sqrt(v.slice<0, 2>() * v.slice<0, 2>()));

	// check x value in v to determine sign
	if (v[0] < 0) {
		result *= -1;
	}

	return result;
}
// [ziquan]
// horizon -> pitch = 0
// downward -> pitch > 0
// otherwise -> pitch < 0
inline static double vectorToPitch(TooN::Vector<3> v) {
	// base vector = (v[0], v[1],0)
	// pitch angle = arccos ( v . (v[0],v[1],0) / |v| * |(v[0],v[1],0)|
	double result = 180.0 / M_PI
			* acos(
					v.slice<0, 2>() * v.slice<0, 2>()
							/ (sqrt(v * v)
									* sqrt(v.slice<0, 2>() * v.slice<0, 2>())));
	if (v[2] > 0) {
		result *= -1;
	}
	return result;
}

// [ziquan]
inline static double angleToValidYaw(double angle) {
	while (angle < -180) angle += 360;
	while (angle >= 180) angle -= 360;
	return angle;
}

#endif /* __HELPERFUNCTIONS_H */
