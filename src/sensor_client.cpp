#include "sensor_client.h"

SensorClient::SensorClient(const EsEkfParams& params, ros::NodeHandle& nh_private)
  : params_(params), es_ekf_(params), start_flag_(true), start_camera_imu_(true),
    start_imu_(true), new_measurement_camera_odom_(false),
    new_measurement_camera_gyro_(false), new_measurement_camera_acc_(false),
    new_measurement_imu_(false), new_measurement_posix_(false),
    imu_(params.model, nh_private)
{
  // TODO makni flag
  std::string acc_topic, gyro_topic, es_ekf_topic;
  nh_private.getParam("acc_topic", acc_topic);
  nh_private.getParam("gyro_topic", gyro_topic);
  nh_private.getParam("es_ekf_topic", es_ekf_topic);

  camera_acc_sub_ =
    node_handle_.subscribe(acc_topic, 1, &SensorClient::camera_acc_callback, this);
  camera_gyro_sub_ =
    node_handle_.subscribe(gyro_topic, 1, &SensorClient::camera_gyro_callback, this);
  estimate_pub_ = node_handle_.advertise<nav_msgs::Odometry>(es_ekf_topic, 1);


  for (int i = 0; i < params_.sensors.size(); ++i) {
    sensor_vec_.push_back(new Sensor(params_.sensors.at(i)));
  }
  while (ros::ok()) {
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    if (((this->camera_imu_ready() && params.use_cam_imu)
         or (!params.use_cam_imu && imu_.isInit()))) {
      int k = 0;
      for (int i = 0; i < params_.sensors.size(); ++i) {
        if (sensor_vec_.at(i)->newMeasurement()) k++;
      }
      if (k == params_.sensors.size()) {
        ROS_INFO("SENSOR INITIALIZED");
        break;
      } else {
        ROS_INFO_STREAM("Waiting for all sensors. Initialized: ["
                        << k << "/" << params_.sensors.size() << "]");
      }

    } else {
      ROS_WARN("Now we wait for sensors...");
      if (ros::isShuttingDown()) break;
    }
  }
  // TODO izvuci van
  update_timer_ =
    node_handle_.createTimer(ros::Duration(0.01), &SensorClient::state_estimation, this);
}

void SensorClient::camera_acc_callback(const sensor_msgs::Imu& msg)
{
  // ROS_INFO("camera_acc_callback");
  if (start_imu_) {
    start_imu_ = false;
    old_time_  = msg.header.stamp.toSec();
    return;
  }
  delta_t_  = msg.header.stamp.toSec() - old_time_;
  old_time_ = msg.header.stamp.toSec();

  new_measurement_camera_acc_ = true;
  camera_acc_.x()             = msg.linear_acceleration.x;
  camera_acc_.y()             = msg.linear_acceleration.y;
  camera_acc_.z()             = msg.linear_acceleration.z;
}

void SensorClient::camera_gyro_callback(const sensor_msgs::Imu& msg)
{
  // ROS_INFO("camera_gyro_callback");
  new_measurement_camera_gyro_ = true;
  camera_gyro_.x()             = msg.angular_velocity.x;
  camera_gyro_.y()             = msg.angular_velocity.y;
  camera_gyro_.z()             = msg.angular_velocity.z;
}

bool SensorClient::outlier_detection(Matrix<double, 3, 1> measurement)
{
  ;
  return ((measurement - es_ekf_.getP()).norm() < params_.outlier_constant);
}


void SensorClient::state_estimation(const ros::TimerEvent& msg)
{
  if (start_flag_) {
    if (sensor_vec_.at(0)->newMeasurement()) {
      es_ekf_.setP(sensor_vec_.at(0)->getPose());
      // TODO if orientaion vector
      es_ekf_.setQ(sensor_vec_.at(0)->getOrientationVector());
      es_ekf_.setV({ 0, 0, 0 });
      start_flag_ = false;
    }
    return;
  }
  // PREDICTION
  bool               prediction  = false;
  bool               measurement = false;
  nav_msgs::Odometry ekf_pose_;
  // TODO ovo makni, provjerava se u inicijalizaciji
  if (!imu_.isInit()) {
    ROS_WARN_THROTTLE(5.0, "IMU not ready");
    return;
  }
  if (params_.use_cam_imu && this->camera_imu_ready()) {
    Matrix<double, 3, 3> cam_imu_to_cam = MatrixXd::Zero(3, 3);
    cam_imu_to_cam(0, 2)                = 1;
    cam_imu_to_cam(1, 0)                = 1;
    cam_imu_to_cam(2, 1)                = 1;
    es_ekf_.prediction(cam_imu_to_cam * this->get_acc(),
                       params_.model.Q_f,
                       cam_imu_to_cam * this->get_angular_vel(),
                       params_.model.Q_w,
                       delta_t_);
    prediction = true;
  } else if (imu_.newMeasurement()) {
    // std::cout << "Linear acc -> " << imu_.get_acc();
    es_ekf_.prediction(imu_.get_acc(),
                       params_.model.Q_f,
                       imu_.get_angular_vel(),
                       params_.model.Q_w,
                       imu_.getDeltaT());
    prediction = true;
  }

  for (int i = 0; i < sensor_vec_.size(); ++i) {
    if (sensor_vec_.at(i)->newMeasurement()) {

      if (sensor_vec_.at(i)->estimateDrift()
          && outlier_detection(sensor_vec_.at(i)->getDriftedPose(es_ekf_.getQDrift(),
                                                                 es_ekf_.getPDrift()))) {
        measurement = true;
        es_ekf_.poseMeasurementUpdateDrift(sensor_vec_.at(i)->getRPose(),
                                           sensor_vec_.at(i)->getPose());

        if (sensor_vec_.at(i)->isOrientationSensor()) {
          es_ekf_.angleMeasurementUpdate(sensor_vec_.at(i)->getROrientation(),
                                         sensor_vec_.at(i)->getOrientation());
        }
        sensor_vec_.at(i)->publishState(true);

      } else if (outlier_detection(sensor_vec_.at(i)->getPose())) {
        measurement = true;
        es_ekf_.poseMeasurementUpdate(sensor_vec_.at(i)->getRPose(),
                                      sensor_vec_.at(i)->getPose());
        sensor_vec_.at(i)->publishState(true);
      } else {
        sensor_vec_.at(i)->publishState(false);
      }
    }

    sensor_vec_.at(i)->publishTransformedPose();
  }

  if (measurement or prediction) {
    Matrix<double, 10, 1> state;
    state = es_ekf_.getState();

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
    ekf_pose_.header.stamp            = ros::Time::now();
    estimate_pub_.publish(ekf_pose_);
  }
}
