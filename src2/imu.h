#ifndef SENSOR_FUSION_IMU_H
#define SENSOR_FUSION_IMU_H
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include <Eigen/Geometry>
#include "error_state_ekf.hpp"


/*
 * Defines the imu subscriber and when an Imu msg comes
 * it calls a model prediction step of the es-ekf.
 */
class Imu{
		ros::Subscriber imu_sub;
		ros::NodeHandle node_handle_;

public:
		Imu(ModelCovariance cov, EsEkf* es_ekf,
			ros::NodeHandle& nh_private):
			init_(false),es_ekf_(es_ekf),
			Q_f_(cov.Q_f),Q_w_(cov.Q_w){
			std::string imu_topic;
			nh_private.getParam("imu_topic", imu_topic);
			imu_sub = node_handle_.subscribe(imu_topic, 1,
																			 &Imu::callback, this);

		}
		void callback(const sensor_msgs::ImuPtr& msg){
			if (!init_){
				ROS_INFO("Imu Prediction: init");
				init_ = true;
				old_time = msg->header.stamp.toSec();
			}
			else if (es_ekf_->isInit()){
				Matrix<double, 3, 1> imu_f,imu_w;
				Matrix<double, 4, 1> imu_angle;
				imu_f << msg->linear_acceleration.x,
								msg->linear_acceleration.y,
								msg->linear_acceleration.z;

				imu_w << msg->angular_velocity.x,
								msg->angular_velocity.y,
								msg->angular_velocity.z;

				double delta_t = msg->header.stamp.toSec() - old_time;
				old_time = msg->header.stamp.toSec();

				es_ekf_->prediction(imu_f,Q_f_,imu_w,Q_w_,delta_t);
			}
			else ROS_INFO("Imu Prediction: Es-Ekf not initialized");
		}

		void setQ(Matrix<double,3,3> Q_f,Matrix<double,3,3> Q_w){
			Q_w_ = Q_w;
			Q_f_ = Q_f;
		}
		bool isInit(){return init_;}
private:
		// Noise matrix
		Matrix<double,3,3> Q_f_, Q_w_;

		// bool fresh measurement
		bool init_;

		double old_time;
		EsEkf* es_ekf_;
};




#endif //SENSOR_FUSION_IMU_H
