#ifndef SENSOR_FUSION_SENSOR_CLIENT_H
#define SENSOR_FUSION_SENSOR_CLIENT_H

#include "error_state_ekf.hpp"
#include "structures.h"

#include "ros/ros.h"
#include <Eigen/Geometry>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Bool.h"
#include "imu.h"
#include "sensor.h"

#include "filter.h"

class SensorClient {
public:
		SensorClient(const EsEkfParams& params,
								ros::NodeHandle& nh_private);

		void camera_acc_callback(const sensor_msgs::Imu& msg);
		void camera_gyro_callback(const sensor_msgs::Imu& msg);
		void camera_odom_callback(const nav_msgs::Odometry& msg);
		void posix_raw_callback(const geometry_msgs::TransformStamped& msg);
		void cartographer_callback();
		void imu_callback(const sensor_msgs::Imu& msg);
		void state_estimation(const ros::TimerEvent& msg);

		bool outlier_detection(Matrix<double,3,1> measurement);

		//Get sensor state
		Matrix<double,3,1> get_acc(){
			if (params_.camera.use_camera_imu){
				new_measurement_camera_acc_ = false;
				return camera_acc_.translation();
			}
			else{
				new_measurement_imu_ = false;
				return imu_acc_.translation();
			}
		}

		Vector3d get_angular_vel(){
			if (params_.camera.use_camera_imu){
				new_measurement_camera_gyro_ = false;
				return camera_gyro_.translation();
			}
			else return imu_gyro_.translation();
		}
		Matrix<double,3,1> get_camera_pose() {
			new_measurement_camera_odom_ = false;
			return (params_.camera.rotation_mat *
							camera_pose_).translation() + params_.camera.translation;
		}

		Matrix<double,3,1> get_camera_lin_vel() {
			return (params_.camera.rotation_mat *
							camera_lin_vel_).translation();
		}

		Matrix<double,4,1> get_camera_orientation(){
			new_measurement_camera_odom_ = false;
			return {camera_orientation_.w(),camera_orientation_.x(),
							camera_orientation_.y(),camera_orientation_.z()};
		}
		Quaterniond get_camera_orientation_quat(){
			new_measurement_camera_odom_ = false;
			return camera_orientation_;
		}

		Matrix<double,3,1> get_pozyx_pose() {
			new_measurement_posix_ = false;
			return (params_.sensors.at(0).rotation_mat *
							posix_pose_).translation() +
							params_.sensors.at(0).translation;
		}

		Translation3d get_cartographer_pose(){
			return cartographer_pose_;
		};
		bool camera_imu_ready(){
			if (new_measurement_camera_gyro_ &&
					new_measurement_camera_acc_){
				return true;
			}
			else return false;
		}

		bool camera_odom_ready(){ return new_measurement_camera_odom_;}
		bool pozyx_ready(){return new_measurement_posix_;}

private:
		ros::Subscriber camera_acc_sub_, camera_gyro_sub_,
										camera_odom_sub_,imu_sub_,
										posix_sub_, cartographer_sub_;

		ros::Publisher estimate_pub_,
										camera_state_pub_,
										pozyx_state_pub_,
										cartographer_state_pub_;
		ros::Timer update_timer_;
		EsEkfParams params_;
		EsEkf2 es_ekf_;
		Imu imu_;
		std::vector<Sensor*> sensor_pointer_vec_;

		ros::NodeHandle node_handle_;
		bool start_flag_,start_imu_,start_camera_imu_;
		bool new_measurement_camera_odom_,
				 new_measurement_posix_,
				 new_measurement_camera_gyro_,
				 new_measurement_camera_acc_,
				 new_measurement_imu_;


		//acceleration and velocities
		Translation3d camera_acc_,camera_gyro_,imu_acc_,imu_gyro_;
		//Poses
		Translation3d camera_pose_,posix_pose_,cartographer_pose_;
		Translation3d camera_lin_vel_;
		Quaterniond camera_orientation_;
		double old_time_,delta_t_;
		double outlier_constant_;

};

#endif //SENSOR_FUSION_SENSOR_CLIENT_H
