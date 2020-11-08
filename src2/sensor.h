#ifndef SENSOR_FUSION_SENSOR_H
#define SENSOR_FUSION_SENSOR_H

#include "error_state_ekf.hpp"
#include "ros/ros.h"
#include "structures.h"
#include <Eigen/Geometry>
#include "nav_msgs/Odometry.h"
using namespace Eigen;

/*
 * We define sensor subscriber and when an odometry msg comes we
 * update the es-ekf state.
 */

class Sensor{
		// subscriber for sensor odometry type msg
		ros::Subscriber sensor_sub;
		ros::NodeHandle node_handle_;
		SensorParams params_;
		EsEkf* es_ekf_;
		bool init;

public:
		Sensor(SensorParams params, EsEkf* es_ekf):
		es_ekf_(es_ekf), params_(params)
		{
			// if its the first measurement update the intial pose
			// and dont call measurement update
			init = false;
			sensor_sub = node_handle_.subscribe(params_.topic, 1,
																					&Sensor::callback_sensor, this);

		}
		bool isInit(){return init;}
		void callback_sensor(const nav_msgs::OdometryPtr& msg){
			Matrix<double, 3, 1> data;
			data << msg->pose.pose.position.x,
							msg->pose.pose.position.y,
							msg->pose.pose.position.z;
			//data = params_.rotation_mat * data + params_.translation;

			if(!init){
				//Set initial state
				//TODO this has to be done in one step with a
				// a defined structure for to es-ekf state.
				// Add tramsformation to all data
				es_ekf_->setPose(data);
				es_ekf_->setVel({msg->twist.twist.linear.x,
												 msg->twist.twist.linear.y,
												 msg->twist.twist.linear.z});
				es_ekf_->setQuat({msg->pose.pose.orientation.w,
													msg->pose.pose.orientation.x,
													msg->pose.pose.orientation.y,
													msg->pose.pose.orientation.z});

				init = true;
			}
			else if(es_ekf_->isInit()){
				es_ekf_->measurement_update(params_.cov.R,
																		data);
			}
			else ROS_INFO("Sensor Measurement: Es-Ekf not initialized");
		}

		~Sensor() {
			std::cout << "SENSOR DESTRUCTOR" << '\n';
			std::string name = "sensor_" + params_.id;
		}
};
#endif