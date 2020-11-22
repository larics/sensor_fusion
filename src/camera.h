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
		ros::Subscriber acc_sub, gyro_sub, odom_sub, imu_sub,pose_sub, gps_sub;
		ros::Publisher odom_pub_,imu_pub_;
		ros::NodeHandle node_handle_;

		EsEkfParams params_;

		EsEkf* es_ekf_;
		bool odom_init_,gyro_init_, gps_init_, acc_init_,posix_init_,first_camera_measurement_;

		Matrix<double, 3, 1> old_pose_,pose_, acc_,angular_vel_,linear_vel_;
		Matrix<double, 4, 1> quat_,old_quat_;

		Matrix<double, 3, 1> pose_posix_,gps_pose_;

		Matrix<double, 3, 3> rotation_matrix_;

		double old_time_;


public:
		Camera(EsEkfParams params, EsEkf* es_ekf,
					 ros::NodeHandle& nh_private):
					 params_(params),es_ekf_(es_ekf),
					 acc_init_(false),gyro_init_(false),
					 odom_init_(false),posix_init_(false),
					 first_camera_measurement_(true),gps_init_(false){

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
			gps_sub = node_handle_.subscribe("/mavros/global_position/local", 1,
																				&Camera::gps_callback, this);

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

			acc_ << msg.linear_acceleration.x,
							msg.linear_acceleration.y,
							msg.linear_acceleration.z;
			acc_ = params_.camera.rotation_mat * acc_;
		}


		void gyro_callback(const sensor_msgs::Imu& msg){
			/*
			 * Obtain the angular velocities measurements
			 */

			angular_vel_ << msg.angular_velocity.x,
											msg.angular_velocity.y,
											msg.angular_velocity.z;

			angular_vel_ = params_.camera.rotation_mat * angular_vel_;
			gyro_init_ = true;
		}

		void odom_callback(const nav_msgs::Odometry& msg){
			/*
			 * Get pose, linear velocities and orientation
			 */
			linear_vel_ << msg.twist.twist.linear.x,
										msg.twist.twist.linear.y,
										msg.twist.twist.linear.z;
			if(params_.camera.is_odom && first_camera_measurement_){
				// first call
				ROS_WARN("odom i nije inicijalizirano");
				old_pose_ << msg.pose.pose.position.x,
										 msg.pose.pose.position.y,
										 msg.pose.pose.position.z;

				old_quat_ << msg.pose.pose.orientation.w,
										 msg.pose.pose.orientation.x,
										 msg.pose.pose.orientation.y,
										 msg.pose.pose.orientation.z;
				first_camera_measurement_ = false;

				es_ekf_->setPose(old_pose_);
				es_ekf_->setQuat(old_quat_);
				es_ekf_->setVel(linear_vel_);
			}

			else if (params_.camera.is_odom && !first_camera_measurement_){
				ROS_WARN("odom i inicijalizirano");
				pose_ << msg.pose.pose.position.x,
								 msg.pose.pose.position.y,
								 msg.pose.pose.position.z;

				quat_ << msg.pose.pose.orientation.w,
								 msg.pose.pose.orientation.x,
								 msg.pose.pose.orientation.y,
								 msg.pose.pose.orientation.z;

				quat_ = quat_ - old_quat_;
				pose_ = pose_ - old_pose_; // delta pose

				old_pose_ << msg.pose.pose.position.x,
								     msg.pose.pose.position.y,
								     msg.pose.pose.position.z;

				old_quat_ << msg.pose.pose.orientation.w,
										 msg.pose.pose.orientation.x,
										 msg.pose.pose.orientation.y,
										 msg.pose.pose.orientation.z;
			}
			else if(!params_.camera.is_odom && !first_camera_measurement_){
				ROS_WARN("nije odom i nije inicijalizirano");
				pose_ << msg.pose.pose.position.x,
								 msg.pose.pose.position.y,
								 msg.pose.pose.position.z;

				quat_ << msg.pose.pose.orientation.w,
								 msg.pose.pose.orientation.x,
								 msg.pose.pose.orientation.y,
								 msg.pose.pose.orientation.z;

				es_ekf_->setPose(pose_);
				es_ekf_->setQuat(quat_);
				es_ekf_->setVel(linear_vel_);

				odom_init_ = true;
				first_camera_measurement_ = false;
			}
			else{
				ROS_WARN("nije odom i inicijalizirano");
				pose_ << msg.pose.pose.position.x,
								msg.pose.pose.position.y,
								msg.pose.pose.position.z;

				quat_ << msg.pose.pose.orientation.w,
								msg.pose.pose.orientation.x,
								msg.pose.pose.orientation.y,
								msg.pose.pose.orientation.z;
				odom_init_ = true;
			}

			//TRANSFORM THE DATA

			pose_ = params_.camera.rotation_mat * pose_ + params_.camera.translation;
			Quaternion<double> Q(quat_(0),quat_(1),
												   quat_(2),quat_(3));
			Matrix3d R;
			R = params_.camera.rotation_mat * Q.toRotationMatrix();
			Q = R;
			quat_ << Q.w(),Q.x(),Q.y(),Q.z();
			linear_vel_ = params_.camera.rotation_mat * linear_vel_;
		}

		void 	pose_callback(const geometry_msgs::TransformStamped& msg){
			posix_init_ = true;
			pose_posix_ << msg.transform.translation.x,
							msg.transform.translation.y,
							msg.transform.translation.z;

			pose_posix_ = params_.sensors.at(0).rotation_mat * pose_posix_;

		}

		void 	gps_callback(const nav_msgs::Odometry& msg){
			gps_init_ = true;
			gps_pose_<< msg.pose.pose.position.x,
												 msg.pose.pose.position.y,
							           msg.pose.pose.position.z;

			pose_posix_ = params_.sensors.at(0).rotation_mat * pose_posix_;

		}

		void imu_callback(const sensor_msgs::Imu& msg){
			ROS_INFO("IMU CALLBACK %f",pose_(0));
			acc_ << msg.linear_acceleration.x,
							msg.linear_acceleration.y,
							msg.linear_acceleration.z;
			angular_vel_ << msg.angular_velocity.x,
											msg.angular_velocity.y,
											msg.angular_velocity.z;
			if (!acc_init_){
				old_time_ = msg.header.stamp.toSec();
				acc_init_ = true;
				return;
			}
			else if(odom_init_ and gps_init_){
				odom_init_ = false; gps_init_ = false;
				double delta_t =  msg.header.stamp.toSec() - old_time_;
				old_time_ = msg.header.stamp.toSec();
				//TODO we need a transform from imu to global

				acc_ = params_.camera.rotation_mat * acc_;
				angular_vel_ = params_.camera.rotation_mat * angular_vel_;

				es_ekf_->prediction(acc_, params_.model.Q_f,
														angular_vel_, params_.model.Q_w,delta_t);

				if (params_.camera.is_odom){
					std::cout << "odom" << std::endl;
					es_ekf_->measurement_update(params_.camera.pose_cov.R,pose_+es_ekf_->getPose());
					es_ekf_->angle_measurement_update(params_.camera.orientation_cov.R,quat_+es_ekf_->getOrientation());
				}
				else{
					std::cout << "nije odom" << std::endl;
					es_ekf_->measurement_update(params_.camera.pose_cov.R,pose_);
					es_ekf_->angle_measurement_update(params_.camera.orientation_cov.R,quat_);
				}
				//es_ekf_->measurement_update(params_.sensors.at(0).cov.R,gps_pose_);
				es_ekf_->velocity_measurement_update(params_.camera.lin_vel_cov.R,
																				 linear_vel_);

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
				ekf_pose_.twist.twist.angular.x   = angular_vel_[0];
				ekf_pose_.twist.twist.angular.y   = angular_vel_[1];
				ekf_pose_.twist.twist.angular.z   = angular_vel_[2];

				ekf_pose_.header.stamp = ros::Time::now();
				odom_pub_.publish(ekf_pose_);
			}
			else{
				double delta_t =  msg.header.stamp.toSec() - old_time_;
				old_time_ = msg.header.stamp.toSec();
				//TODO here we prediction, lowest rate of publishing

				acc_ = params_.camera.rotation_mat * acc_;
				angular_vel_ = params_.camera.rotation_mat * angular_vel_;

				es_ekf_->prediction(acc_, params_.model.Q_f,
														angular_vel_, params_.model.Q_w,
														delta_t);

			}
			return;
		}

};

#endif //SENSOR_FUSION_CAMERA_H
