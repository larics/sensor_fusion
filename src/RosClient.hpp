#include "Sensor.hpp"
#include "mav_msgs/Actuators.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/LinkStates.h>


using std::cos;
using std::sin;

class RosClient{

  ros::NodeHandle node_handle_;
  ros::Subscriber motor_sub;
  ros::Subscriber sensor_sub;
  ros::Subscriber sensor_2_sub;
  ros::Subscriber base_link_pose_sub;
  ros::Publisher pose_pub;
  Ekf ekf;

  public:
  RosClient(VehicleParams params,std::vector<SensorParams> sensor_params)
  :ekf(params){
    T_ = params.T;
    std::string motor_speed_topic,imu_topic;
    node_handle_.getParam("motor_speed_topic", motor_speed_topic);
    node_handle_.getParam("imu_topic", imu_topic);
    node_handle_.getParam("motor_speed_topic", motor_speed_topic);

    motor_sub = node_handle_.subscribe(motor_speed_topic, 1000,
                            &RosClient::callback_motor, this);
    pose_pub = node_handle_.advertise<geometry_msgs::Pose>
                        ("ekf/ekf_pose", 1);
    base_link_pose_sub = node_handle_.subscribe("/gazebo/link_states",1000,
                            &RosClient::callback_gazebo,this);

    motor_speed_ << 0,0,0,0;
    old_pose_for_odom_ << 0,0,0;


    sensor_obj_.reserve(3);
    for (size_t i = 0; i < sensor_params.size(); i++) {
      sensor_obj_.push_back(new Sensor(sensor_params.at(i)));
    }
    std::cout << sensor_obj_.at(0)->getID() << '\n';
    std::cout << sensor_obj_.at(1)->getID() << '\n';

    std::cout << sensor_obj_.at(0)->getTopic() << '\n';
    std::cout << sensor_obj_.at(1)->getTopic() << '\n';

  }

  void callback_motor(const mav_msgs::ActuatorsPtr& msg){
    motor_speed_ << msg->angular_velocities[0],msg->angular_velocities[1],
                    msg->angular_velocities[2],msg->angular_velocities[3];

  }

  void callback_gazebo(const gazebo_msgs::LinkStatesPtr& msg){

    pose_gazebo_.x = msg->pose[1].position.x;
    pose_gazebo_.y = msg->pose[1].position.y;
    pose_gazebo_.z = msg->pose[1].position.z;
  }

  void update_dynamics(){
    ros::Rate loop_rate(1.0/T_);
    while (ros::ok())
    {
      Pose p;

      p = ekf.prediction_step(motor_speed_);

      for (size_t i = 0; i < sensor_obj_.size(); i++) {

          if (sensor_obj_.at(i)->isOdomSensor()){
            p = ekf.measurment_update(sensor_obj_.at(i)->getSensorData() + old_pose_for_odom_,
                                      sensor_obj_.at(i)->getR());
          }
          else{
            p = ekf.measurment_update(sensor_obj_.at(i)->getSensorData(),
                                      sensor_obj_.at(i)->getR());
          }

        }
      old_pose_for_odom_ << p.x,p.y,p.z;
      ekf_pose_.position.x = p.x;
      ekf_pose_.position.y = p.y;
      ekf_pose_.position.z = p.z;

      poses_ekf_.x.push_back(p.x);
      poses_ekf_.y.push_back(p.y);
      poses_ekf_.z.push_back(p.z);

      pose_truth_.x.push_back(pose_gazebo_.x);
      pose_truth_.y.push_back(pose_gazebo_.y);
      pose_truth_.z.push_back(pose_gazebo_.z);

      std::cout << "Pose with model \nX: " << p.x << '\n'
                << "Y: " << p.y << '\n'
                << "Z: " << p.z << '\n';
      pose_pub.publish(ekf_pose_);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  ~RosClient(){
    for (size_t i = 0; i < sensor_obj_.size(); i++) {
      sensor_obj_.at(i)->~Sensor();
    }
    std::string name = "ekf";
    save_vector_as_matrix(name,poses_ekf_);
    name = "truth";
    save_vector_as_matrix(name,pose_truth_);
  }

  double T_;
  Eigen::Matrix<double, 1, 4> motor_speed_;

  Pose pose_gazebo_;

  std::vector<Sensor*> sensor_obj_;
  geometry_msgs::Pose ekf_pose_;
  Eigen::Matrix<double, 3, 1> old_pose_for_odom_;
  Pose_vec pose_truth_;
  Pose_vec poses_ekf_;

};
