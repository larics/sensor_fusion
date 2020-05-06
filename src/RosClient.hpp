#include <Eigen/Geometry>

#include "ros/ros.h"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkStates.h>
//#include "vehicle_dynamics.hpp"
#include "ekf.hpp"
using std::cos;
using std::sin;

class RosClient{

  ros::NodeHandle node_handle_;
  ros::Subscriber motor_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber sensor_sub;
  ros::Subscriber sensor_2_sub;
  ros::Subscriber base_link_pose_sub;
  ros::Publisher pose_pub;
  Ekf ekf;
  //VehicleDynamics vehicle;

  public:
  RosClient(VehicleParams params):ekf(params){

    std::cout << "/* message */" << motor_speed_ << '\n';
    sensor_sub = node_handle_.subscribe("/uav/odometry2", 1000,
                            &RosClient::callback_sensor, this);
    sensor_2_sub = node_handle_.subscribe("/uav/odometry3", 1000,
                                &RosClient::callback_sensor_2, this);
    motor_sub = node_handle_.subscribe("/uav/motor_speed", 1000,
                            &RosClient::callback_motor, this);
    imu_sub = node_handle_.subscribe("/uav/imu",1000,
                            &RosClient::callback_imu,this);
    pose_pub = node_handle_.advertise<geometry_msgs::Pose>
                        ("ekf/ekf_pose", 1);
    base_link_pose_sub = node_handle_.subscribe("/gazebo/link_states",1000,
                            &RosClient::callback_gazebo,this);

    motor_speed_ << 0,0,0,0;
    sensor << 0,0,0;

  }

  void callback_motor(const mav_msgs::ActuatorsPtr& msg){
    motor_speed_ << msg->angular_velocities[0],msg->angular_velocities[1],
                    msg->angular_velocities[2],msg->angular_velocities[3];

  }

  void callback_gazebo(const gazebo_msgs::LinkStatesPtr& msg){

    pose_gazebo.x = msg->pose[1].position.x;
    pose_gazebo.y = msg->pose[1].position.y;
    pose_gazebo.z = msg->pose[1].position.z;
  }

  void callback_sensor(const nav_msgs::OdometryPtr& msg){
    sensor << msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z;
  }

  void callback_sensor_2(const nav_msgs::OdometryPtr& msg){
  sensor_2 << msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z;
  }

  void callback_imu(const sensor_msgs::ImuPtr& msg){
      Quaternions q;
      q.x = msg->orientation.x;
      q.y = msg->orientation.y;
      q.z = msg->orientation.z;
      q.w = msg->orientation.w;

      Orientation_ = ToEulerAngles(q);
    }

  void update_dynamics(){
    ros::Rate loop_rate(25);
    while (ros::ok())
    {
      Pose p;
      p = ekf.prediction_step(sensor,sensor_2,motor_speed_);
      geometry_msgs::Pose pose;
      pose.position.x = p.x;
      pose.position.y = p.y;
      pose.position.z = p.z;

      pose_ekf.x.push_back(p.x);
      pose_ekf.y.push_back(p.y);
      pose_ekf.z.push_back(p.z);

      pose_truth.x.push_back(pose_gazebo.x);
      pose_truth.y.push_back(pose_gazebo.y);
      pose_truth.z.push_back(pose_gazebo.z);

      pose_sensor.x.push_back(sensor(0));
      pose_sensor.y.push_back(sensor(1));
      pose_sensor.z.push_back(sensor(2));

      pose_sensor_2.x.push_back(sensor_2(0));
      pose_sensor_2.y.push_back(sensor_2(1));
      pose_sensor_2.z.push_back(sensor_2(2));

      std::cout << "Pose with model \nX: " << p.x << '\n'
                << "Y: " << p.y << '\n'
                << "Z: " << p.z << '\n';
      pose_pub.publish(pose);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  ~RosClient(){
    std::string name = "ekf";
    save_vector_as_matrix(name,pose_ekf);
    name = "truth";
    save_vector_as_matrix(name,pose_truth);
    name = "sensor";
    save_vector_as_matrix(name,pose_sensor);
    name = "sensor2";
    save_vector_as_matrix(name,pose_sensor_2);
  }


  Eigen::Matrix<double, 1, 4> motor_speed_;
  Eigen::Matrix<double, 3, 1> sensor;
  Eigen::Matrix<double, 3, 1> sensor_2;
  EulerAngles Orientation_;

  Pose pose_gazebo;


  Pose_vec pose_truth;
  Pose_vec pose_ekf;
  Pose_vec pose_sensor;
  Pose_vec pose_sensor_2;

};
