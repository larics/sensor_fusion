#include "imu.h"

Imu::Imu(ros::NodeHandle& nh_private){
	std::string imu_topic;
	nh_private.getParam("imu_topic", imu_topic);
	imu_sub = node_handle_.subscribe(imu_topic, 1000,
																	 &Imu::callback, this);
	for (int i = 0; i < 6; ++i) {
		R_(i,i) = 1;
	}
	};

void Imu::callback(const sensor_msgs::Imu& msg){
	new_measurement = true;
	// the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
	tf::Quaternion quat;
	tf::quaternionMsgToTF(msg.orientation, quat);

	// the tf::Quaternion has a method to acess roll pitch and yaw
	double roll, pitch, yaw;
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	data_(0) = roll;
	data_(1) = pitch;
	data_(2) = yaw;
	data_(3) = msg.angular_velocity.x;
	data_(4) = msg.angular_velocity.y;
	data_(5) = msg.angular_velocity.z;
}