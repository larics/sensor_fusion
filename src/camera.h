#ifndef SENSOR_FUSION_CAMERA_H
#define SENSOR_FUSION_CAMERA_H

#include "ros/ros.h"
#include "error_state_ekf.hpp"
#include "structures.h"
#include <Eigen/Geometry>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Imu.h"

class Camera{
private:
		ros::Subscriber acc_sub, gyro_sub, odom_sub, imu_sub,pose_sub;
		ros::Publisher odom_pub_,imu_pub_;
		ros::NodeHandle node_handle_;

		CameraParams params_;

		EsEkf* es_ekf_;
		bool odom_init_,gyro_init_, acc_init_;

		Matrix<double, 3, 1> old_pose_,pose_, acc_,angular_vel_,linear_vel_;
		Matrix<double, 4, 1> quat_;

		Matrix<double, 3, 1> pose_posix_;

		Matrix<double, 3, 3> rotation_matrix_;

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

			std::cout << acc_topic << odom_topic << gyro_topic << std::endl;

			imu_sub = node_handle_.subscribe("/mavros/imu/data", 1,
																			 &Camera::imu_callback, this);

			acc_sub = node_handle_.subscribe(acc_topic, 1,
																			 &Camera::acc_callback, this);
			odom_sub = node_handle_.subscribe(odom_topic, 1,
																			 &Camera::odom_callback, this);
			gyro_sub = node_handle_.subscribe(gyro_topic, 1,
																			 &Camera::gyro_callback, this);
			pose_sub = node_handle_.subscribe("dummy", 1,
																				&Camera::pose_callback, this);

			odom_pub_ = node_handle_.advertise<nav_msgs::Odometry>
									("ekf_odom", 1);

			imu_pub_ = node_handle_.advertise<sensor_msgs::Imu>
							("imu_transformed", 1);

			//Rotation matrix from gyroscope and accelerometer to camera frame
			rotation_matrix_ = MatrixXd::Zero(3,3);
			rotation_matrix_(0,2) = 1;
			rotation_matrix_(1,0) = 1;
			rotation_matrix_(2,1) = 1;
			std::cout << rotation_matrix_ << "\n";
			ros::spin();
		}

		void acc_callback(const sensor_msgs::Imu& msg){
			/*
			 * Here we get acceleration measurements from the camera
			 * It has the lowest rate so we call the prediction here.
			 */

			std::cout << "ovo je ovo "<< acc_init_ <<  odom_init_ << gyro_init_ << std::endl;
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
				Matrix<double, 3, 3> R = MatrixXd::Identity(3,3);

				acc_ = rotation_matrix_ * acc_;
				angular_vel_ = rotation_matrix_*angular_vel_;

				es_ekf_->prediction(acc_, 0.01*R,
												angular_vel_, 0.01*R, delta_t);

				es_ekf_->measurement_update(0.01*R,pose_);
				es_ekf_->angle_measurement_update(0.01*R,quat_);
				es_ekf_->velocity_measurement_update(R,linear_vel_);

				Matrix<double,10,1> state = es_ekf_->getState();
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
				ekf_pose_.twist.twist.angular.x = angular_vel_[0];
				ekf_pose_.twist.twist.angular.y = angular_vel_[1];
				ekf_pose_.twist.twist.angular.z = angular_vel_[2];

				ekf_pose_.header.stamp = ros::Time::now();
				odom_pub_.publish(ekf_pose_);


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

			if(! odom_init_){
				linear_vel_ << msg.twist.twist.linear.x,
								msg.twist.twist.linear.y,
								msg.twist.twist.linear.z;
				pose_ << msg.pose.pose.position.x,
								msg.pose.pose.position.y,
								msg.pose.pose.position.z;

				quat_ << msg.pose.pose.orientation.w,
								msg.pose.pose.orientation.x,
								msg.pose.pose.orientation.y,
								msg.pose.pose.orientation.z;


				old_pose_ << msg.pose.pose.position.x,
								msg.pose.pose.position.y,
								msg.pose.pose.position.z;
				es_ekf_->setQuat(quat_);
				es_ekf_->setVel(linear_vel_);
				es_ekf_->setPose(pose_);
				odom_init_ = true;
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
				odom_init_ = true;
			}

			else{
				pose_ << msg.pose.pose.position.x,
								 msg.pose.pose.position.y,
								 msg.pose.pose.position.z;

				quat_ << msg.pose.pose.orientation.w,
								 msg.pose.pose.orientation.x,
								 msg.pose.pose.orientation.y,
								 msg.pose.pose.orientation.z;
				odom_init_ = true;
			}
			linear_vel_ << msg.twist.twist.linear.x,
										 msg.twist.twist.linear.y,
										 msg.twist.twist.linear.z;
			pose_ << msg.pose.pose.position.x,
							msg.pose.pose.position.y,
							msg.pose.pose.position.z;

			quat_ << msg.pose.pose.orientation.w,
							msg.pose.pose.orientation.x,
							msg.pose.pose.orientation.y,
							msg.pose.pose.orientation.z;
			odom_init_ = true;

		}
		void imu_callback(const sensor_msgs::Imu& msg){
			Matrix<double,3,1> acc,vel;
			acc <<  msg.linear_acceleration.x,
							msg.linear_acceleration.y,
							msg.linear_acceleration.z;

			vel << msg.angular_velocity.x,
							msg.angular_velocity.y,
							msg.angular_velocity.z;

			Matrix<double,3,3> rot;
			Matrix<double,3,1> translation;
			translation << -0.00171191,-0.00013322,-0.00013322;
			rot << -0.48273718, 0.83398352, 0.26727571,
							0.00869194, -0.30061343, 0.95370646,
							0.87572214, 0.46271271, 0.13786835;
			Quaternion<double> quat(msg.orientation.w,msg.orientation.x,
													    msg.orientation.y,msg.orientation.z);
			acc = rot.transpose()*acc;
			vel = rot.transpose()*vel;
			Matrix<double,3,3> rot_quat;
			rot_quat = rot.transpose()*quat.toRotationMatrix();
			std::cout << "rotation \n" << rot.transpose() << std::endl;
			Quaternion<double> quat_rotatated(rot_quat);
			//std::cout << "quat \n" << quat_rotatated << std::endl;
			sensor_msgs::Imu data;
			data.angular_velocity.x = vel[0];
			data.angular_velocity.y = vel[1];
			data.angular_velocity.z = vel[2];

			data.linear_acceleration.x = acc[0];
			data.linear_acceleration.y = acc[1];
			data.linear_acceleration.z = acc[2];


			data.orientation.w = quat_rotatated.w();
			data.orientation.x = quat_rotatated.x();
			data.orientation.y = quat_rotatated.y();
			data.orientation.z = quat_rotatated.z();


			data.header.stamp = ros::Time::now();
			imu_pub_.publish(data);
		}

		void 	pose_callback(const geometry_msgs::TransformStamped& msg){
			pose_posix_ << 0,0,0;
		}



};

#endif //SENSOR_FUSION_CAMERA_H
