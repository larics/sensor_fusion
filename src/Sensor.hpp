#include "RosClient.hpp"



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

  void callback_sensor(const nav_msgs::OdometryPtr& msg){
    sensor_ << msg->pose.pose.position.x,
               msg->pose.pose.position.y,
               msg->pose.pose.position.z;
    //ROS_INFO("stamp: %f", params_.R(0,0));
    std::cout << "id" << params_.id << '\n';
  }
  SensorParams params_;
  Eigen::Matrix<double, 3, 1> sensor_;
  };
