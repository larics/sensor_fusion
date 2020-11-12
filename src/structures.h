#ifndef SENSOR_FUSION_STRUCTURES_H
#define SENSOR_FUSION_STRUCTURES_H
#include <Eigen/Geometry>
#include "vector"
#include "nav_msgs/Odometry.h"
#include <iostream>

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

		Matrix<double, 3, 3> rotation_mat;
		Matrix<double, 3, 1> translation;

		SensorCovariance cov; // correlation matrix of the sensor
};

struct CameraParams{
		Matrix<double, 3, 3> rotation_mat;
		Matrix<double, 3, 1> translation;
		//TODO add bias
		bool is_odom;
		SensorCovariance cov;
};

struct EsEkfParams{
		std::vector<SensorParams> sensors;
		ModelCovariance model;
};

class EsEkfState{
private:
		Matrix<double, 10, 1> state;
public:
		EsEkfState(){
			state = MatrixXd::Zero(10,1);
		}
		Matrix<double, 10, 1> getState() const{return state;}
		void operator=(const nav_msgs::OdometryPtr& msg){
			this->state << msg->pose.pose.position.x,
										 msg->pose.pose.position.y,
										 msg->pose.pose.position.z,
										 msg->twist.twist.linear.x,
										 msg->twist.twist.linear.y,
										 msg->twist.twist.linear.z,
										 msg->pose.pose.orientation.w,
										 msg->pose.pose.orientation.z,
										 msg->pose.pose.orientation.y,
										 msg->pose.pose.orientation.z;
		}
		friend std::ostream& operator<<(std::ostream& out, const EsEkfState& f);
};
std::ostream& operator<<(std::ostream& out, const EsEkfState& f)
{
	out << f.getState().transpose() << "\n";
}

#endif //SENSOR_FUSION_STRUCTURES_H