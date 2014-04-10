#ifndef QUATEULER_H
#define QUATEULER_H

#include <math.h> 

#include <Eigen/Geometry>

enum Rotation{
// Rotation sequence:
// ZYX: rotation on GlobalX follow by GlobalY follow by GlobalZ
// XYZ: rotation on GlobalZ follow by GlobalY follow by GlobalZ
	ZYX, XYZ
};


void threeaxisrot(double r11, double r12, double r21, double r31, double r32, double res[])
{
    res[0] = atan2( r11, r12 ); 
    res[1] = asin ( r21 );		
    res[2] = atan2( r31, r32 ); 
}

// functions to return euler angle from quaternion

void quaternion2Euler(const Eigen::Quaternion<double>& q,  double res[], Rotation r=ZYX){  
	switch (r)
	{
	case ZYX:
		threeaxisrot( 2*(q.x()*q.y() + q.w()*q.z()), 
						q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(), 
						-2*(q.x()*q.z() - q.w()*q.y()), 
						2*(q.y()*q.z() + q.w()*q.x()), 
						q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(), res);
		break;
	 

	case XYZ:
		threeaxisrot( -2*(q.y()*q.z() - q.w()*q.x()), 
                        q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z(), 
                        2*(q.x()*q.z() + q.w()*q.y()), 
                        -2*(q.x()*q.y() - q.w()*q.z()), 
						q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z(), res);
        
		break;
	}
}

#endif