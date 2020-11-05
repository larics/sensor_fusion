#include "mav_msgs/Actuators.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkStates.h>
#include "mavros_msgs/RCOut.h"
#include <tf2/LinearMath/Quaternion.h>
#include "imu.h"
#include "ros/ros.h"


using std::cos;
using std::sin;

class ImuRosClient{

		ros::NodeHandle nh_private_;
		ros::NodeHandle node_handle_;
		ros::Subscriber sensor_sub;
		ros::Subscriber imu_sub;
		ros::Publisher pose_pub;
		ros::Publisher odom_pub;
		ros::Timer update_timer;
public:
		ImuRosClient(){
			T_ = 0.05;
			i = 0;

//			pose_pub = node_handle_.advertise<sensor_msgs::Imu>
//							("imu_new", 1);
			odom_pub = node_handle_.advertise<nav_msgs::Odometry>
							("/uav/odom_new", 1);
//			imu_sub = node_handle_.subscribe("/uav/imu", 1,
//																			 &ImuRosClient::imu_callback, this);
			sensor_sub = node_handle_.subscribe("/uav/odometry2", 1,
																					&ImuRosClient::callback_sensor, this);
			update_timer = node_handle_.createTimer(ros::Duration(T_), &ImuRosClient::update_imu,this);
			ros::spin();
		}

//		void imu_callback(const sensor_msgs::Imu& msg){
//			imu_data_ = msg;
//			std::cout << " tu sam " << std::endl;
//		}
		void callback_sensor(const nav_msgs::Odometry& msg){
			sensor_data_ = msg;
		}

		void update_imu(const ros::TimerEvent& msg){
			odom_pub.publish(sensor_data_);

		}

		sensor_msgs::Imu imu_data_;
		nav_msgs::Odometry sensor_data_;
		int i;
		double T_;
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "rescan");
 ImuRosClient();
}
