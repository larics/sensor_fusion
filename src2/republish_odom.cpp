#ifndef SENSOR_FUSION_REPUBLISH_ODOM_H
#define SENSOR_FUSION_REPUBLISH_ODOM_H

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"


using std::cos;
using std::sin;

class ImuRosClient{

		ros::NodeHandle nh_private_;
		ros::NodeHandle node_handle_;
		ros::Subscriber sensor_sub;
		ros::Publisher odom_pub;
		ros::Timer update_timer;
public:
		ImuRosClient(){
			T_ = 0.05;
			
			odom_pub = node_handle_.advertise<nav_msgs::Odometry>
							("/uav/odom_new", 1);
			sensor_sub = node_handle_.subscribe("/uav/odometry2", 1,
																					&ImuRosClient::callback_sensor, this);
			update_timer = node_handle_.createTimer(ros::Duration(T_), &ImuRosClient::update_imu,this);
			ros::spin();
		}

		void callback_sensor(const nav_msgs::Odometry& msg){
			sensor_data_ = msg;
		}

		void update_imu(const ros::TimerEvent& msg){
			odom_pub.publish(sensor_data_);

		}

		nav_msgs::Odometry sensor_data_;
		double T_;
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "rescan");
	ImuRosClient();
}

#endif //SENSOR_FUSION_REPUBLISH_ODOM_H
