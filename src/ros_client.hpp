//#include "error_state_ekf.hpp"
//#include "sensor.h"
//#include "imu.h"
//
//#include <dynamic_reconfigure/server.h>
//#include <sensor_fusion/EKF_QConfig.h>
///*
// * Define and initialize all relevant subscribers and publishers,
// * create a dynamic reconfigure server for sensor and model noise.
// *
// * Wait for the measurements before we start
// */
//
//class RosClient {
//public:
//		RosClient(EsEkfParams params,
//						  ros::NodeHandle& nh_private):
//						  nh_private_(nh_private),es_ekf_(),
//						  imu_(params.model,&es_ekf_,nh_private){
//
//			std::string es_ekf_topic;
//			nh_private_.getParam("es_ekf_topic", es_ekf_topic);
//			pose_pub = node_handle_.advertise<nav_msgs::Odometry>
//							("ekf_odom", 1);
//
//			sensor_obj_.reserve(3);
//			for (size_t i = 0; i < params.sensors.size(); i++) {
//				sensor_obj_.push_back(new Sensor( params.sensors.at(i),&es_ekf_));
//			}
//
//			dynamic_reconfigure::Server<sensor_fusion::EKF_QConfig> server;
//			dynamic_reconfigure::Server<sensor_fusion::EKF_QConfig>::CallbackType f;
//
//			f = boost::bind(&RosClient::dynamic_reconfigure_callback,
//											this, _1, _2);
//			server.setCallback(f);
//
//			/*
//			 * Waits until all measurements are received then sets the
//			 * Es-Ekf to initialized and sets up the timed update for regular
//			 * state estimation publishing
//			 */
//			while (true){
//				int k = 0;
//				for (size_t i = 0; i < params.sensors.size(); i++) {
//					if(sensor_obj_.at(i)->isInit()) { k++;}
//				}
//
//				if (k == params.sensors.size() && imu_.isInit()){
//					es_ekf_.setInit();
//					update_timer = node_handle_.createTimer
//									(ros::Duration(0.05),
//										&RosClient::publish_state,this);
//					break;
//				}
//				ROS_INFO("Now we wait for sensors...");
//				ros::spinOnce();
//				ros::Duration(0.1).sleep();
//			}
//			ros::spin();
//		}
//
//		void dynamic_reconfigure_callback(sensor_fusion::EKF_QConfig &config, uint32_t level) {
//
//			Matrix3d Q_f = MatrixXd::Zero(3,3);
//			Q_f(0,0) = config.Q_acc_x;
//			Q_f(1,1) = config.Q_acc_y;
//			Q_f(2,2) = config.Q_acc_z;
//			Matrix3d Q_w = MatrixXd::Zero(3,3);
//			Q_w(0,0) = config.Q_angular_x;
//			Q_w(1,1) = config.Q_angular_y;
//			Q_w(2,2) = config.Q_angular_z;
//			imu_.setQ(Q_f,Q_w);
//			Matrix3d R = MatrixXd::Zero(3,3);
//			R(0,0) = config.R_px;
//			R(1,1) = config.R_py;
//			R(2,2) = config.R_pz;
//			sensor_obj_.at(0)->setR(R);
//		}
//
//		void publish_state(const ros::TimerEvent& msg){
//			Matrix<double,10,1> state = es_ekf_.getState();
//			nav_msgs::Odometry ekf_pose_;
//			ekf_pose_.pose.pose.position.x = state[0];
//			ekf_pose_.pose.pose.position.y = state[1];
//			ekf_pose_.pose.pose.position.z = state[2];
//
//			ekf_pose_.twist.twist.linear.x = state[3];
//			ekf_pose_.twist.twist.linear.y = state[4];
//			ekf_pose_.twist.twist.linear.z = state[5];
//
//			ekf_pose_.pose.pose.orientation.w = state[6];
//			ekf_pose_.pose.pose.orientation.x = state[7];
//			ekf_pose_.pose.pose.orientation.y = state[8];
//			ekf_pose_.pose.pose.orientation.z = state[9];
//			ekf_pose_.twist.twist.angular.x = -1;
//			ekf_pose_.twist.twist.angular.y = -1;
//			ekf_pose_.twist.twist.angular.z = -1;
//
//			ekf_pose_.header.stamp = ros::Time::now();
//			pose_pub.publish(ekf_pose_);
//		}
//private:
//		ros::NodeHandle nh_private_;
//		ros::NodeHandle node_handle_;
//		ros::Publisher pose_pub;
//		ros::Timer update_timer;
//
//		// Define relevant objects
//		Imu imu_;
//		EsEkf es_ekf_;
//		// Sensor vector so we can define number of sensors in runtime
//		std::vector<Sensor*> sensor_obj_;
//
//};