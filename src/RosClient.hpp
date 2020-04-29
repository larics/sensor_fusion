#include <Eigen/Geometry>

#include "ros/ros.h"
#include "mav_msgs/Actuators.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/Imu.h>
#include "vehicle_dynamics.hpp"
using std::cos;
using std::sin;

class RosClient{

  ros::NodeHandle node_handle_;
  ros::Subscriber motor_sub;
  ros::Subscriber imu_sub;
  ros::Publisher pose_pub;
  VehicleDynamics vehicle;

  public:
  RosClient(){

    std::cout << "/* message */" << motor_speed_ << '\n';
    motor_sub = node_handle_.subscribe("/uav/motor_speed", 1000,
                            &RosClient::callback_motor, this);
    imu_sub = node_handle_.subscribe("/uav/imu",1000,
                            &RosClient::callback_imu,this);
    pose_pub = node_handle_.advertise<geometry_msgs::Pose>
                        ("ekf/model_pose", 1);
  }

  void callback_motor(const mav_msgs::ActuatorsPtr& msg){
    // std::cout << "sub1-> "<< motor_speed_ << '\n';
    motor_speed_ << msg->angular_velocities[0],msg->angular_velocities[1],
                    msg->angular_velocities[2],msg->angular_velocities[3];
    //
    // std::cout << "motor speed -->  " << motor_speed_ << '\n';
  }

  void callback_imu(const sensor_msgs::ImuPtr& msg){
      Quaternions q;
      q.x = msg->orientation.x;
      q.y = msg->orientation.y;
      q.z = msg->orientation.z;
      q.w = msg->orientation.w;

      Orientation_ = ToEulerAngles(q);
      // std::cout << "Orientation \n"
      //           << Orientation_.roll << "\n"
      //           << Orientation_.pitch << "\n"
      //           << Orientation_.yaw <<'\n';
    }

  void update_dynamics(){
    ros::Rate loop_rate(25);
    while (ros::ok())
    {
      Pose p;
      p = vehicle.integrate_step(Orientation_.roll,
                                 Orientation_.pitch,
                                 Orientation_.yaw,motor_speed_);
      geometry_msgs::Pose pose;
      pose.position.x = p.x;
      pose.position.y = p.y;
      pose.position.z = p.z;
      std::cout << "Pose with model \nX: " << p.x << '\n'
                << "Y: " << p.y << '\n'
                << "Z: " << p.z << '\n';
      pose_pub.publish(pose);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  

  Eigen::Matrix<double, 1, 4> motor_speed_;
  EulerAngles Orientation_;
};
