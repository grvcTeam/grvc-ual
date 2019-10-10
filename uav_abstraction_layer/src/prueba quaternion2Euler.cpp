///
#include <stdio.h>
#include <string.h>
#include "Quat.h"
#include <cmath>



typedef double Quaterniond [4];
// Quaterniond quaternion;


Quaterniond toQuaternion(double pitch, double roll, double yaw)
{
	Quaterniond q;
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w() = cy * cr * cp + sy * sr * sp;
	q.x() = cy * sr * cp - sy * cr * sp;
	q.y() = cy * cr * sp + sy * sr * cp;
	q.z() = sy * cr * cp - cy * sr * sp;
	return q;
}


//

static void toEulerAngle(const Quaterniond& q, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
	yaw = atan2(siny, cosy);
}

