#ifndef SENSOR_FUSION_STRUCTURES_H
#define SENSOR_FUSION_STRUCTURES_H
#include <Eigen/Geometry>
#include "vector"

using namespace Eigen;

/*
 * Here we define 2 model covariance matrices.
 * -> Q_f is the model noise we get when we integrate
 * linear acceleration
 * -> Q_w model noise from angular velocity
 *
 * essentially imu noise for acceleration and
 * angular velocity measurement
 */
struct ModelCovariance{
		Matrix<double,3,3> Q_f, Q_w;
		ModelCovariance(){
			Q_f = MatrixXd::Zero(3,3);
			Q_w = MatrixXd::Zero(3,3);
		}
};

struct SensorCovariance{
		Matrix<double,3,3> R;
		SensorCovariance(){
			R = MatrixXd::Zero(3,3);
		}
};
struct SensorParams{
		std::string id;
		std::string topic;
		// 1 if its an odom sensor, gives delta values not absolute values
		bool is_odom;

		double w_x, w_y, w_z; // rotation
		double d_x, d_y, d_z; // translation

		SensorCovariance cov; // correlation matrix of the sensor
};

struct EsEkfParams{
		std::vector<SensorParams> sensors;
		ModelCovariance model;
};

#endif //SENSOR_FUSION_STRUCTURES_H
