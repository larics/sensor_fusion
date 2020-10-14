#include "Sensor.hpp"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkStates.h>
#include "ekf.hpp"
#include "mavros_msgs/RCOut.h"
#include <tf2/LinearMath/Quaternion.h>
#include "imu.h"

#include <dynamic_reconfigure/server.h>
#include <sensor_fusion/EKF_QConfig.h>

using std::cos;
using std::sin;

class RosClient{

  ros::NodeHandle nh_private_;
  ros::NodeHandle node_handle_;
  ros::Subscriber motor_sub;
  ros::Subscriber sensor_sub;
  ros::Subscriber sensor_2_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber base_link_pose_sub;
  ros::Publisher pose_pub;
  ros::Timer update_timer;
  Ekf ekf;

  //EkfImu ekf_imu;

  public:
  RosClient(VehicleParams params,std::vector<SensorParams> sensor_params,
						ros::NodeHandle& nh_private,
						std::vector<double> R_imu)
  :ekf(params), nh_private_(nh_private),imu_(nh_private,R_imu) {
    T_ = params.T;
    std::string motor_speed_topic,imu_topic;
    nh_private_.getParam("motor_speed_topic", motor_speed_topic);
    nh_private_.getParam("imu_topic", imu_topic);
    nh_private_.getParam("motor_speed_topic", motor_speed_topic);

    motor_sub = node_handle_.subscribe(motor_speed_topic, 1000,
                            &RosClient::callback_motor, this);
    pose_pub = node_handle_.advertise<nav_msgs::Odometry>
                        ("ekf_odom", 1000);
		imu_sub = node_handle_.subscribe(imu_topic, 1000,
																		 &RosClient::callback_imu, this);
    base_link_pose_sub = node_handle_.subscribe("/gazebo/link_states",1000,
                            &RosClient::callback_gazebo,this);


    motor_speed_ << 0,0,0,0;
    old_pose_for_odom_ << 0,0,0;
		acc_ << 0,0,0;

    sensor_obj_.reserve(3);
    for (size_t i = 0; i < sensor_params.size(); i++) {
      sensor_obj_.push_back(new Sensor(sensor_params.at(i)));
    }
		dynamic_reconfigure::Server<sensor_fusion::EKF_QConfig> server;
		dynamic_reconfigure::Server<sensor_fusion::EKF_QConfig>::CallbackType f;

		f = boost::bind(&RosClient::dynamic_reconfigure_callback,
									this, _1, _2);
		server.setCallback(f);

    int k = 0;
    while (true){
			for (size_t i = 0; i < sensor_params.size(); i++) {
				if(sensor_obj_.at(i)->freshMeasurement()) k++;
			}
			if (k == sensor_params.size() && imu_.newMeasurement()) {
				Eigen::Matrix<double, 12, 1> state;
				state << sensor_obj_.at(0)->getInitialState();
				ekf.setInitialState(state);
				update_timer = node_handle_.createTimer(ros::Duration(T_), &RosClient::update_dynamics,this);
				break;
			}
			ros::spinOnce();
    }
		ros::spin();
  }

	void dynamic_reconfigure_callback(sensor_fusion::EKF_QConfig &config, uint32_t level) {

		Eigen::Matrix<double, 12, 12> Q;
		Q = MatrixXd::Zero(12, 12);
		Q(3,3) = config.Qx*T_;
		Q(4,4) = config.Qy*T_;
		Q(5,5) = config.Qz*T_;
		Q(9,9) = config.Q_roll*T_;
		Q(10,10) = config.Q_pitch*T_;
		Q(11,11) = config.Q_yaw*T_;

		ekf.setQ(Q);
  }

  void callback_imu(const sensor_msgs::Imu& msg){
  	acc_[0] = msg.linear_acceleration.x;
		acc_[1] = msg.linear_acceleration.y;
		acc_[2] = msg.linear_acceleration.z - 9.81;
  }

  void callback_motor(const mavros_msgs::RCOutPtr& msg){
    motor_speed_ << msg->channels[0],msg->channels[1],
                    msg->channels[2],msg->channels[3];

  }

  void callback_gazebo(const gazebo_msgs::LinkStatesPtr& msg){

    pose_gazebo_.x = msg->pose[1].position.x;
    pose_gazebo_.y = msg->pose[1].position.y;
    pose_gazebo_.z = msg->pose[1].position.z;
  }

  void update_dynamics(const ros::TimerEvent& msg){
    Pose p;

    p = ekf.prediction_step(motor_speed_);

    if(imu_.newMeasurement()){
    	p = ekf.imu_measurment_update(imu_.getImuData(),
															 imu_.getR());
    }

    for (size_t i = 0; i < sensor_obj_.size(); i++) {

    	if (sensor_obj_.at(i)->freshMeasurement()){
				//if (sensor_obj_.at(i)->isOdomSensor()){
				//  p = ekf.measurment_update(sensor_obj_.at(i)->getSensorData() + old_pose_for_odom_,
				//                            sensor_obj_.at(i)->getR());
				//}
				//else{

					p = ekf.measurment_update(sensor_obj_.at(i)->getSensorData(),
																		sensor_obj_.at(i)->getR());

				// Create odometry msg
    	}
    }

		ekf_pose_.pose.pose.position.x = p.x;
		ekf_pose_.pose.pose.position.y = p.y;
		ekf_pose_.pose.pose.position.z = p.z;
		ekf_pose_.twist.twist.linear.x = p.x_dot;
		ekf_pose_.twist.twist.linear.y = p.y_dot;
		ekf_pose_.twist.twist.linear.z = p.z_dot;
		tf2::Quaternion quat;
		quat.setRPY(p.x_angle,p.y_angle,p.z_angle);
		ekf_pose_.pose.pose.orientation.w = quat.w();
		ekf_pose_.pose.pose.orientation.x = quat.x();
		ekf_pose_.pose.pose.orientation.y = quat.y();
		ekf_pose_.pose.pose.orientation.z = quat.z();
		ekf_pose_.twist.twist.angular.x = p.x_angular;
		ekf_pose_.twist.twist.angular.y = p.y_angular;
		ekf_pose_.twist.twist.angular.z = p.z_angular;

		ekf_pose_.header.stamp = ros::Time::now();
		pose_pub.publish(ekf_pose_);




    std::cout << "Pose with model \nX: " << p.x<< '\n'
              << "Y: " << p.y << '\n'
              << "Z: " << p.z << '\n';
  }

  ~RosClient(){
    for (size_t i = 0; i < sensor_obj_.size(); i++) {
      sensor_obj_.at(i)->~Sensor();
    }
  }

  double T_;
  Eigen::Matrix<double, 1, 4> motor_speed_;

  Pose pose_gazebo_;

  Imu imu_;
  std::vector<Sensor*> sensor_obj_;
  nav_msgs::Odometry ekf_pose_;
  Eigen::Matrix<double, 3, 1> old_pose_for_odom_, acc_;
  Pose_vec pose_truth_;
  Pose_vec poses_ekf_;

};
