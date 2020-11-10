#ifndef SENSOR_FUSION_CAMERA_H
#define SENSOR_FUSION_CAMERA_H

#include "ros/ros.h"
#include "error_state_ekf.hpp"
#include "structures.h"
#include <Eigen/Geometry>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

class Camera{
private:
		ros::Subscriber acc_sub, gyro_sub, odom_sub;
		ros::NodeHandle node_handle_;

		CameraParams params_;

		EsEkf* es_ekf_;
		bool odom_init_,gyro_init_, acc_init_;

		Matrix<double, 3, 1> old_pose_,pose_, acc_,angular_vel_,linear_vel_;
		Matrix<double, 4, 1> quat_;

		double old_time_;


public:
		Camera(CameraParams params, EsEkf* es_ekf,
					 ros::NodeHandle& nh_private):
					 params_(params),es_ekf_(es_ekf),
					 acc_init_(false),gyro_init_(false),odom_init_(false){

			std::string acc_topic,odom_topic,gyro_topic;
			nh_private.getParam("acc_topic", acc_topic);
			nh_private.getParam("odom_topic", odom_topic);
			nh_private.getParam("gyro_topic", gyro_topic);

			acc_sub = node_handle_.subscribe(acc_topic, 1,
																			 &Camera::acc_callback, this);
			odom_sub = node_handle_.subscribe(odom_topic, 1,
																			 &Camera::odom_callback, this);
			gyro_sub = node_handle_.subscribe(gyro_topic, 1,
																			 &Camera::gyro_callback, this);

		}

		void acc_callback(const sensor_msgs::Imu& msg){
			/*
			 * Here we get acceleration measurements from the camera
			 * It has the lowest rate so we call the prediction here.
			 */


			acc_ << msg.linear_acceleration.x,
							msg.linear_acceleration.y,
							msg.linear_acceleration.z;
			if (!acc_init_){
				old_time_ = msg.header.stamp.toSec();
				acc_init_ = true;
			}
			else if(odom_init_ and gyro_init_){
				double delta_t =  msg.header.stamp.toSec() - old_time_;
				old_time_ = msg.header.stamp.toSec();
				//TODO here we prediction, lowest rate of publishing
			}
		}


		void gyro_callback(const sensor_msgs::Imu& msg){
			/*
			 * Obtain the angular velocities measurements
			 */
		//TODO transform the data to match camera output

			angular_vel_ << msg.angular_velocity.x,
											msg.angular_velocity.y,
											msg.angular_velocity.z;
			gyro_init_ = true;
		}

		void odom_callback(const nav_msgs::Odometry& msg){
			/*
			 * Get pose, linear velocities and orientation
			 */

			if(!odom_init_ && params_.is_odom){
				old_pose_ << msg.pose.pose.position.x,
								msg.pose.pose.position.y,
								msg.pose.pose.position.z;
			}
			else if (params_.is_odom){

				pose_ << msg.pose.pose.position.x - old_pose_[0],
								 msg.pose.pose.position.y - old_pose_[1],
								 msg.pose.pose.position.z - old_pose_[2];

				//TODO for quaternions
				quat_ << msg.pose.pose.orientation.w,
								msg.pose.pose.orientation.x,
								msg.pose.pose.orientation.y,
								msg.pose.pose.orientation.z;

				old_pose_ << msg.pose.pose.position.x,
										 msg.pose.pose.position.y,
										 msg.pose.pose.position.z;
			}

			else{
				pose_ << msg.pose.pose.position.x,
								 msg.pose.pose.position.y,
								 msg.pose.pose.position.z;

				quat_ << msg.pose.pose.orientation.w,
								 msg.pose.pose.orientation.x,
								 msg.pose.pose.orientation.y,
								 msg.pose.pose.orientation.z;
			}
			linear_vel_ << msg.twist.twist.linear.x,
										 msg.twist.twist.linear.y,
										 msg.twist.twist.linear.z;

		}



};

#endif //SENSOR_FUSION_CAMERA_H
