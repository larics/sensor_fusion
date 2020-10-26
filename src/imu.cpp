#include "imu.h"

Imu::Imu(ros::NodeHandle& nh_private,
				 std::vector<double> R_imu,
				 EsEkf* es_ekf): es_ekf_(es_ekf),
				 new_measurement(false),first_measurement(true){
	std::string imu_topic;
	nh_private.getParam("imu_topic", imu_topic);
	imu_sub = node_handle_.subscribe(imu_topic, 1000,
																	 &Imu::callback, this);
	for (int i = 0; i < 6; ++i) {
		R_(i,i) = R_imu.at(i);
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
	if (first_measurement){
		old_time = msg.header.stamp.toSec();
		first_measurement = false;
	}
	else{
		Matrix<double, 3, 1> imu_f,imu_w;
		imu_f << msg.linear_acceleration.x,
						msg.linear_acceleration.y,
						msg.linear_acceleration.z;

		imu_w << msg.angular_velocity.x,
						msg.angular_velocity.y,
						msg.angular_velocity.z;

		Matrix<double, 3, 3> var_imu_f,var_imu_w;
		double delta_t = msg.header.stamp.toSec() - old_time;
		var_imu_f = R_.block<3,3>(0,0);
		var_imu_f = R_.block<3,3>(3,0);
		es_ekf_->predicition(imu_f,var_imu_f,imu_w,var_imu_w,delta_t);
	}
}