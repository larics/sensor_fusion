#ifndef SENSOR_FUSION_EKF_IMU_H
#define SENSOR_FUSION_EKF_IMU_H
//https://www.kalmanfilter.net/stateextrap.html

#include <iostream>
#include <Eigen/Geometry>
#include <cmath>
#include "utils.hpp"

using std::cos;
using std::sin;
using std::fabs;
using namespace Eigen;

class EkfImu {
public:
		EkfImu(VehicleParams params);
		void predictionStep(Eigen::Matrix<double, 3, 1> acc);
		Pose measurmentUpdate(Eigen::Matrix<double, 3, 1> sensor,
													Eigen::Matrix<double, 3, 3> R);

private:
		Eigen::Matrix<double, 6, 1> state; // x, y, z, x_dot, y_dot, z_dot
		Eigen::Matrix<double, 6, 6> phi; // state = phi*state + gamma*U // U is linear acc
		Eigen::Matrix<double, 6, 3> gamma;
		Eigen::Matrix<double, 6, 6> Q;
		Eigen::Matrix<double, 3, 6> H; //observation matrix
		Eigen::Matrix<double, 6, 3> K; //Kalman gain
		Eigen::Matrix<double, 6, 6> P_minus;
		Eigen::Matrix<double, 6, 6> P;
};


#endif //SENSOR_FUSION_EKF_IMU_H
