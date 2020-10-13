#ifndef SENSOR_FUSION_IMU_H
#define SENSOR_FUSION_IMU_H

#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include <Eigen/Geometry>
#include "tf/transform_datatypes.h"

class Imu{
		ros::Subscriber imu_sub;
		ros::NodeHandle node_handle_;

	public:
		Imu(ros::NodeHandle& nh_private,
			std::vector<double> R_imu);
		void callback(const sensor_msgs::Imu& msg);
		Eigen::Matrix<double, 6, 1> getImuData(){
			new_measurement = false;
			return data_;}
		Eigen::Matrix<double, 6, 6> getR(){return R_;}
		bool newMeasurement(){return new_measurement;};


	private:
		// Angular velocity and orientatrion
		Eigen::Matrix<double, 6, 1> data_;
		// Noise matrix
		Eigen::Matrix<double, 6, 6>  R_;
		// bool fresh measurement
		bool new_measurement;
};


#endif //SENSOR_FUSION_IMU_H
