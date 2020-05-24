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
   sensor_ << 0,0,0;
  }

  std::string getID(){return params_.id; }

  std::string getTopic(){return params_.topic; }

  Eigen::Matrix<double, 3, 3> getR(){return params_.R; }

  Eigen::Matrix<double, 3, 1> getSensorData(){
    pose_sensor_.x.push_back(sensor_(0));
    pose_sensor_.y.push_back(sensor_(1));
    pose_sensor_.z.push_back(sensor_(2));
    
    return sensor_;}

  bool isOdomSensor(){
    return params_.is_odom;}


  void callback_sensor(const nav_msgs::OdometryPtr& msg){
    sensor_ << msg->pose.pose.position.x,
               msg->pose.pose.position.y,
               msg->pose.pose.position.z;
  }
  ~Sensor(){
    std::cout << "SENSOR DESTRUCTOR" << '\n';
    std::string name = "sensor_"+params_.id;
    save_vector_as_matrix(name,pose_sensor_);
  }


private:
  SensorParams params_;
  Pose_vec pose_sensor_;
  Eigen::Matrix<double, 3, 1> sensor_;
  };
