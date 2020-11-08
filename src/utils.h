#ifndef SENSOR_FUSION_UTILS_H
#define SENSOR_FUSION_UTILS_H

#include <Eigen/Geometry>

using namespace Eigen;
// Utilities for the sensor fusion algorithm

static const double pi = 3.14159265358979323846;
//! Wrap angles to [-pi, pi)
/*!
    Some discussion about alternative implementations:
    https://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi
 */
inline double wrapToPi(double angle) {
	// Naive solution assumes that we're wrapping regularly,
	// So that the angle argument will never be much bigger than 2pi
	// or much smaller than -2pi

	while (angle >= pi) angle -= 2 * pi;
	while (angle < -pi) angle += 2 * pi;

	return angle;
}

Matrix<double, 3, 3> skew_symetric(Matrix<double,3,1> m){
	Matrix<double, 3, 3> a;
	a << 0, -m(2),m(1),
					m(2),0,-m(0),
					-m(1),m(0),0;
	return a;
}

Matrix<double, 4, 1> axixs_angle2quat(Matrix<double, 3, 1> axis_angle){
	double norm = axis_angle.norm();
	double w,x,y,z;
	w = cos(norm / 2);
	if (norm < 1e-10){
		x = 0;
		y = 0;
		z = 0;
	}
	else{
		x = axis_angle[0]/norm * sin(norm/2);
		y = axis_angle[1]/norm * sin(norm/2);
		z = axis_angle[2]/norm * sin(norm/2);
	}
	Matrix<double, 4, 1> result;
	result << w,x,y,z;
	return result;
}

Quaternion<double> euler2quat(Matrix<double,3,1> euler){

	double roll = euler(0);
	double pitch = euler(1);
	double yaw = euler(2);

	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	double w = cr * cp * cy + sr * sp * sy;
	double x = sr * cp * cy - cr * sp * sy;
	double y = cr * sp * cy + sr * cp * sy;
	double z = cr * cp * sy - sr * sp * cy;

	return Quaternion<double>(w,x,y,z);
}



#endif //SENSOR_FUSION_UTILS_H
