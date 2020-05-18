#include <iostream>
#include "ros/ros.h"
#include <Eigen/Geometry>
#include "ekf.hpp"

#include "nav_msgs/Odometry.h"

using namespace Eigen;

class Sensor{
  ros::Subscriber sensor_sub;
  ros::NodeHandle node_handle_;

  public:
  Sensor(SensorParams params){
    params_ = params;
    sensor_sub = node_handle_.subscribe(params_.topic, 1000,
                            &Sensor::callback_sensor, this);
    std::cout << "id: " <<params_.id << '\n'
              << "topic: \"" <<params_.topic << "\"\n"
              << "R: \n" << params_.R << "\n\n";
  }

  std::string getID(){return params_.id; }

  std::string getTopic(){return params_.topic; }

  Eigen::Matrix<double, 3, 3> getR(){return params_.R; }

  Eigen::Matrix<double, 3, 1> getSensorData(){return sensor_;}


  void callback_sensor(const nav_msgs::OdometryPtr& msg){
    sensor_ << msg->pose.pose.position.x,
               msg->pose.pose.position.y,
               msg->pose.pose.position.z;
  }


private:
  SensorParams params_;
  Eigen::Matrix<double, 3, 1> sensor_;
  };
