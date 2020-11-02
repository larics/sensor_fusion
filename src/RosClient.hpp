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
  :ekf(params),es_ekf() ,nh_private_(nh_private),
  imu_(nh_private,R_imu,&es_ekf){
    T_ = params.T;
    std::string motor_speed_topic,imu_topic;
    nh_private_.getParam("motor_speed_topic", motor_speed_topic);
    nh_private_.getParam("imu_topic", imu_topic);
    nh_private_.getParam("motor_speed_topic", motor_speed_topic);

//    motor_sub = node_handle_.subscribe(motor_speed_topic, 1000,
//                            &RosClient::callback_motor, this);
    pose_pub = node_handle_.advertise<nav_msgs::Odometry>
                        ("ekf_odom", 1000);
    base_link_pose_sub = node_handle_.subscribe("/gazebo/link_states",1000,
                            &RosClient::callback_gazebo,this);


    motor_speed_ << 0,0,0,0;
    old_pose_for_odom_ << 0,0,0;

    sensor_obj_.reserve(3);
    for (size_t i = 0; i < sensor_params.size(); i++) {
      sensor_obj_.push_back(new Sensor(sensor_params.at(i),&es_ekf));
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
		Eigen::Matrix<double, 6, 6> R = MatrixXd::Zero(6,6);
		R(0,0) =  config.Rx;
		R(1,1) =  config.Ry;
		R(2,2) =  config.Rz;
		R(3,3) =  config.R_xdot;
		R(4,4) =  config.R_ydot;
		R(5,5) =  config.R_zdot;

		sensor_obj_.at(0)->setR(R);

  }

//  void callback_motor(const mavros_msgs::RCOutPtr& msg){
//    motor_speed_ << msg->channels[0],msg->channels[1],
//                    msg->channels[2],msg->channels[3];
//
//  }

  void callback_gazebo(const gazebo_msgs::LinkStatesPtr& msg){

    pose_gazebo_.x = msg->pose[1].position.x;
    pose_gazebo_.y = msg->pose[1].position.y;
    pose_gazebo_.z = msg->pose[1].position.z;
  }

  void update_dynamics(const ros::TimerEvent& msg){

  	Matrix<double,10,1> state = es_ekf.getState();

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
		ekf_pose_.twist.twist.angular.x = -1;
		ekf_pose_.twist.twist.angular.y = -1;
		ekf_pose_.twist.twist.angular.z = -1;

		ekf_pose_.header.stamp = ros::Time::now();
		pose_pub.publish(ekf_pose_);




//    std::cout << "Pose with model \nX: " << state[0] <<'\n'
//              << "Y: " << state[1] <<'\n'
//              << "Z: " << state[2] <<'\n';
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
  EsEkf es_ekf;
  std::vector<Sensor*> sensor_obj_;
  nav_msgs::Odometry ekf_pose_;
  Eigen::Matrix<double, 3, 1> old_pose_for_odom_;

	// error state Ekf
	Eigen::Matrix<double, 3, 1> g;
	Eigen::Matrix<double, 9, 6>	l_jac;
	Eigen::Matrix<double, 3, 9>	h_jac;
	Eigen::Matrix<double, 3, 1>	ekf_pose, ekf_velocity;
	Eigen::Matrix<double, 4, 1>	quat;
	Eigen::Matrix<double, 9, 9> p_cov;

	double old_time; //seconds

};
