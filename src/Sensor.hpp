#include <iostream>
#include "ros/ros.h"
#include <Eigen/Geometry>
#include "ekf_imu.h"

#include "nav_msgs/Odometry.h"

using namespace Eigen;

class Sensor{
  ros::Subscriber sensor_sub;
  ros::NodeHandle node_handle_;

  public:
  Sensor(SensorParams params){
    params_ = params;
    new_data = false;
    sensor_sub = node_handle_.subscribe(params_.topic, 1000,
                            &Sensor::callback_sensor, this);

    rotation_ = RotationMatrix(params.w_x, params.w_y, params.w_z);
    translation_ << params.d_x, params.d_y, params.d_z;

    std::cout << "id: " <<params_.id << '\n'
						  << "odom: " <<params_.is_odom << '\n'
              << "topic: \"" <<params_.topic << "\"\n"
              << "R: \n" << params_.R << "\n\n";
   sensor_ << 0,0,0;
  }

  std::string getID(){return params_.id; }

  std::string getTopic(){return params_.topic; }
  bool freshMeasurement(){return new_data; }

  Eigen::Matrix<double, 6, 6> getR(){return params_.R; }

  Eigen::Matrix<double, 6, 1> getSensorData(){
    pose_sensor_.x.push_back(sensor_(0));
    pose_sensor_.y.push_back(sensor_(1));
    pose_sensor_.z.push_back(sensor_(2));
		double T = sensor_data_.header.stamp.toSec() - old_sensor_data_.header.stamp.toSec();
		std::cout << "time " << T << std::endl;
		Eigen::Matrix<double, 3, 1> old_sensor;
		old_sensor << old_sensor_data_.pose.pose.position.x,
									old_sensor_data_.pose.pose.position.y,
									old_sensor_data_.pose.pose.position.z;
		Eigen::Matrix<double, 3, 1> speed = (sensor_-old_sensor)/T;
		if (T == 0) speed << 0,0,0;
		old_sensor_data_ = sensor_data_;
		Eigen::Matrix<double, 6, 1> data;
		data << sensor_data_.pose.pose.position.x,
						sensor_data_.pose.pose.position.y,
						sensor_data_.pose.pose.position.z,
						speed[0],speed[1],speed[2];
		new_data = false;
    //return rotation_*sensor_ + translation_;
    return data;
  }

		Eigen::Matrix<double, 3, 1> getRawSensorData(){
  		// Sensor data in the sensor coordinate system

			return sensor_;
		}

  bool isOdomSensor(){
    return params_.is_odom;}


  void callback_sensor(const nav_msgs::OdometryPtr& msg){
    sensor_ << msg->pose.pose.position.x,
               msg->pose.pose.position.y,
               msg->pose.pose.position.z;
    sensor_data_ = *msg;
    new_data = true;
    std::cout << "SENSOR" << std::endl;
  }
  ~Sensor(){
    std::cout << "SENSOR DESTRUCTOR" << '\n';
    std::string name = "sensor_"+params_.id;
    save_vector_as_matrix(name,pose_sensor_);
  }


private:
  SensorParams params_;
  Pose_vec pose_sensor_;
  nav_msgs::Odometry sensor_data_, old_sensor_data_;
  Eigen::Matrix<double, 3, 1> sensor_;
  Eigen::Matrix<double, 3, 3> rotation_;
  Eigen::Matrix<double, 3, 1> translation_;
  bool new_data;

  };
