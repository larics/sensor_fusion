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
  ros::Subscriber base_link_pose_sub;
  ros::Publisher pose_pub;
  Ekf ekf;
  //VehicleDynamics vehicle;

  public:
  RosClient(VehicleParams params):ekf(params){

    std::cout << "/* message */" << motor_speed_ << '\n';
    sensor_sub = node_handle_.subscribe("/uav/odometry", 1000,
                            &RosClient::callback_sensor, this);
    motor_sub = node_handle_.subscribe("/uav/motor_speed", 1000,
                            &RosClient::callback_motor, this);
    imu_sub = node_handle_.subscribe("/uav/imu",1000,
                            &RosClient::callback_imu,this);
    pose_pub = node_handle_.advertise<geometry_msgs::Pose>
                        ("ekf/ekf_pose", 1);
    base_link_pose_sub = node_handle_.subscribe("/gazebo/link_states",1000,
                            &RosClient::callback_gazebo,this);

  }

  void callback_motor(const mav_msgs::ActuatorsPtr& msg){
    // std::cout << "sub1-> "<< motor_speed_ << '\n';
    motor_speed_ << msg->angular_velocities[0],msg->angular_velocities[1],
                    msg->angular_velocities[2],msg->angular_velocities[3];
    //
    // std::cout << "motor speed -->  " << motor_speed_ << '\n';
  }

  void callback_gazebo(const gazebo_msgs::LinkStatesPtr& msg){
    x_truth.push_back(msg->pose[1].position.x);
    y_truth.push_back(msg->pose[1].position.y);
    z_truth.push_back(msg->pose[1].position.z);
  }

  void callback_sensor(const nav_msgs::OdometryPtr& msg){
    sensor << msg->pose.pose.position.x,
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
      p = ekf.prediction_step(sensor,motor_speed_);
      geometry_msgs::Pose pose;
      pose.position.x = p.x;
      pose.position.y = p.y;
      pose.position.z = p.z;
      x.push_back(p.x);
      y.push_back(p.y);
      z.push_back(p.z);
      std::cout << "Pose with model \nX: " << p.x << '\n'
                << "Y: " << p.y << '\n'
                << "Z: " << p.z << '\n';
      pose_pub.publish(pose);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  ~RosClient(){
    const std::string file_namex = "x.mat";
    const std::string file_namez = "z.mat";
    const std::string file_namey = "y.mat";
    const std::string vec_namex = "x";
    const std::string vec_namey = "y";
    const std::string vec_namez = "z";
    std::ofstream file(file_namex);
    save_vector_as_matrix( vec_namex ,x , file );
    std::ofstream filey(file_namey);
    save_vector_as_matrix( vec_namey ,y , filey );
    std::ofstream filez(file_namez);
    save_vector_as_matrix( vec_namez ,z , filez );

    const std::string file_namex_truth = "x_truth.mat";
    const std::string file_namez_truth = "z_truth.mat";
    const std::string file_namey_truth = "y_truth.mat";
    const std::string vec_namex_truth = "x";
    const std::string vec_namey_truth = "y";
    const std::string vec_namez_truth = "z";
    std::ofstream file_truth(file_namex_truth);
    save_vector_as_matrix( vec_namex_truth ,x_truth , file_truth );
    std::ofstream filey_truth(file_namey_truth);
    save_vector_as_matrix( vec_namey_truth ,y_truth , filey_truth );
    std::ofstream filez_truth(file_namez_truth);
    save_vector_as_matrix( vec_namez_truth ,z_truth , filez_truth );
  }


  Eigen::Matrix<double, 1, 4> motor_speed_;
  Eigen::Matrix<double, 3, 1> sensor;
  EulerAngles Orientation_;

  std::vector<double> x_truth;
  std::vector<double> y_truth;
  std::vector<double> z_truth;
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;

};
