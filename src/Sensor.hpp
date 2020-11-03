#include <iostream>
#include "ros/ros.h"
#include <Eigen/Geometry>
#include "error_state_ekf.hpp"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"

using namespace Eigen;

class Sensor{
  ros::Subscriber sensor_sub;
  ros::NodeHandle node_handle_;

  public:
  Sensor(SensorParams params,EsEkf* es_ekf_):es_ekf(es_ekf_){
    params_ = params;
    new_data = false;
		first_measurement = false;
    sensor_sub = node_handle_.subscribe(params_.topic, 1,
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
  void setR(const Eigen::Matrix<double, 6, 6>& R){params_.R = R;}

  Eigen::Matrix<double, 12, 1> getInitialState(){
		// the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
		tf::Quaternion quat;
		tf::quaternionMsgToTF(sensor_data_.pose.pose.orientation, quat);

		// the tf::Quaternion has a method to acess roll pitch and yaw
		double roll, pitch, yaw;
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

		Eigen::Matrix<double, 12, 1> data;
		data << sensor_data_.pose.pose.position.x,
						sensor_data_.pose.pose.position.y,
						sensor_data_.pose.pose.position.z,
						sensor_data_.twist.twist.linear.x,
						sensor_data_.twist.twist.linear.y,
						sensor_data_.twist.twist.linear.z,
						roll,pitch,yaw,
						sensor_data_.twist.twist.angular.x,
						sensor_data_.twist.twist.angular.y,
						sensor_data_.twist.twist.angular.z;

		return data;
  };


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
    //std::cout << "SENSOR" << std::endl;
		if (es_ekf->isIinit() or first_measurement){
			first_measurement = false;
			es_ekf->measurement_update(params_.R.block<3,3>(0,0),
																 sensor_);
		}
		else{
			es_ekf->setPest(sensor_);
		}

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
  bool new_data,first_measurement;

  EsEkf* es_ekf;
  };
