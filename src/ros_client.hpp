#include "error_state_ekf.hpp"
#include "sensor.h"
#include "imu.h"
/*
 * Define and initialize all relevant subscribers and publishers,
 * create a dynamic reconfigure server for sensor and model noise.
 *
 * Wait for the measurements before we start
 */

class RosClient {
public:
		RosClient(EsEkfParams params,
						  ros::NodeHandle& nh_private):
						  nh_private_(nh_private),es_ekf_(),
						  imu_(params.model,&es_ekf_,nh_private){

			std::string es_ekf_topic;
			nh_private_.getParam("es_ekf_topic", es_ekf_topic);
			pose_pub = node_handle_.advertise<nav_msgs::Odometry>
							("ekf_odom", 1);

			sensor_obj_.reserve(3);
			for (size_t i = 0; i < params.sensors.size(); i++) {
				sensor_obj_.push_back(new Sensor( params.sensors.at(i),&es_ekf_));
			}

			/*
			 * Waits until all measurements are received then sets the
			 * Es-Ekf to initialized and sets up the timed update for regular
			 * state estimation publishing
			 */
			while (true){
				int k = 0;
				for (size_t i = 0; i < params.sensors.size(); i++) {
					if(sensor_obj_.at(i)->isInit()) { k++;}
				}

				if (k == params.sensors.size() && imu_.isInit()){
					es_ekf_.setInit();
					update_timer = node_handle_.createTimer
									(ros::Duration(0.05),
										&RosClient::publish_state,this);
					break;
				}
				ROS_INFO("Now we wait...");
				ros::spinOnce();
				ros::Duration(0.1).sleep();
			}
			ROS_INFO("All is well");
			ros::spin();
		}

		void publish_state(const ros::TimerEvent& msg){
			Matrix<double,10,1> state = es_ekf_.getState();
			nav_msgs::Odometry ekf_pose_;
			ekf_pose_.pose.pose.position.x = state[0];
			ekf_pose_.pose.pose.position.y = state[1];
			ekf_pose_.pose.pose.position.z = state[2];

			ekf_pose_.twist.twist.linear.x = state[3];
			ekf_pose_.twist.twist.linear.y = state[4];
			ekf_pose_.twist.twist.linear.z = state[5];

			ekf_pose_.pose.pose.orientation.w = state[6];
			ekf_pose_.pose.pose.orientation.x = state[7];
			ekf_pose_.pose.pose.orientation.y = state[8];
			ekf_pose_.pose.pose.orientation.z = state[9];
			ekf_pose_.twist.twist.angular.x = -1;
			ekf_pose_.twist.twist.angular.y = -1;
			ekf_pose_.twist.twist.angular.z = -1;

			ekf_pose_.header.stamp = ros::Time::now();
			pose_pub.publish(ekf_pose_);
		}
private:
		ros::NodeHandle nh_private_;
		ros::NodeHandle node_handle_;
		ros::Publisher pose_pub;
		ros::Timer update_timer;

		// Define relevant objects
		Imu imu_;
		EsEkf es_ekf_;
		// Sensor vector so we can define number of sensors in runtime
		std::vector<Sensor*> sensor_obj_;

};