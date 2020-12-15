#include "sensor_client.h"

SensorClient::SensorClient(const EsEkfParams& params,
                           ros::NodeHandle& nh_private)
    : params_(params),
      es_ekf_(params),
      start_flag_(true),
      start_camera_imu_(true),
      start_imu_(true),
      new_measurement_camera_odom_(false),
      new_measurement_camera_gyro_(false),
      new_measurement_camera_acc_(false),
      new_measurement_imu_(false),
      new_measurement_posix_(false),
      imu_(params.model, nh_private),
      outlier_constant_(params.outlier_constant) {
  std::string acc_topic, camera_odom_topic, gyro_topic, imu_topic, posix_topic,
      es_ekf_topic;
  nh_private.getParam("acc_topic", acc_topic);
  nh_private.getParam("odom_topic", camera_odom_topic);
  nh_private.getParam("gyro_topic", gyro_topic);
  nh_private.getParam("imu_topic", imu_topic);
  nh_private.getParam("posix_topic", posix_topic);
  nh_private.getParam("es_ekf_topic", es_ekf_topic);

  imu_sub_ = node_handle_.subscribe("/mavros/imu/data", 1,
                                    &SensorClient::imu_callback, this);

  camera_acc_sub_ = node_handle_.subscribe(
      acc_topic, 1, &SensorClient::camera_acc_callback, this);
  camera_odom_sub_ = node_handle_.subscribe(
      camera_odom_topic, 1, &SensorClient::camera_odom_callback, this);
  camera_gyro_sub_ = node_handle_.subscribe(
      gyro_topic, 1, &SensorClient::camera_gyro_callback, this);
  posix_sub_ = node_handle_.subscribe(posix_topic, 1,
                                      &SensorClient::posix_raw_callback, this);

  estimate_pub_ = node_handle_.advertise<nav_msgs::Odometry>(es_ekf_topic, 1);

  pozyx_state_pub_ = node_handle_.advertise<std_msgs::Bool>("/pozyx_state", 1);
  camera_state_pub_ =
      node_handle_.advertise<std_msgs::Bool>("/camera_state", 1);

  while (true) {
    if (((this->camera_imu_ready() && params.camera.use_camera_imu) or
         (!params.camera.use_camera_imu && new_measurement_imu_)) &&
        this->camera_odom_ready() && new_measurement_posix_) {
      ROS_INFO("SENSOR INITIALIZED");
      break;
    } else {
      ROS_WARN("Now we wait for sensors...");
      ros::Duration(0.25).sleep();
      ros::spinOnce();
      if (ros::isShuttingDown()) break;
    }
  }

  update_timer_ = node_handle_.createTimer(
      ros::Duration(0.01), &SensorClient::state_estimation, this);

  ROS_INFO("Starting sensor client");
  ros::MultiThreadedSpinner spinner(0);  // Use max number of threads
  spinner.spin();  // spin() will not return until the node has been shutdown
}

void SensorClient::camera_acc_callback(const sensor_msgs::Imu& msg) {
  // ROS_INFO("camera_acc_callback");
  if (start_imu_) {
    start_imu_ = false;
    old_time_ = msg.header.stamp.toSec();
    return;
  }
  delta_t_ = msg.header.stamp.toSec() - old_time_;
  old_time_ = msg.header.stamp.toSec();

  new_measurement_camera_acc_ = true;
  camera_acc_.x() = msg.linear_acceleration.x;
  camera_acc_.y() = msg.linear_acceleration.y;
  camera_acc_.z() = msg.linear_acceleration.z;
}

void SensorClient::camera_gyro_callback(const sensor_msgs::Imu& msg) {
  // ROS_INFO("camera_gyro_callback");
  new_measurement_camera_gyro_ = true;
  camera_gyro_.x() = msg.angular_velocity.x;
  camera_gyro_.y() = msg.angular_velocity.y;
  camera_gyro_.z() = msg.angular_velocity.z;
}

void SensorClient::camera_odom_callback(const nav_msgs::Odometry& msg) {
  // ROS_INFO("camera_odom_callback");
  new_measurement_camera_odom_ = true;
  camera_pose_.x() = msg.pose.pose.position.x;
  camera_pose_.y() = msg.pose.pose.position.y;
  camera_pose_.z() = msg.pose.pose.position.z;

  camera_orientation_.w() = msg.pose.pose.orientation.w;
  camera_orientation_.x() = msg.pose.pose.orientation.x;
  camera_orientation_.y() = msg.pose.pose.orientation.y;
  camera_orientation_.z() = msg.pose.pose.orientation.z;

  camera_lin_vel_.x() = msg.twist.twist.linear.x;
  camera_lin_vel_.y() = msg.twist.twist.linear.y;
  camera_lin_vel_.z() = msg.twist.twist.linear.z;
}

void SensorClient::posix_raw_callback(
    const geometry_msgs::TransformStamped& msg) {
  // ROS_INFO("camera_posix_callback");
  new_measurement_posix_ = true;
  posix_pose_.x() = msg.transform.translation.x;
  posix_pose_.y() = msg.transform.translation.y;
  posix_pose_.z() = msg.transform.translation.z;
}

void SensorClient::cartographer_callback() {
  ROS_INFO("cartographer_callback");
}

void SensorClient::imu_callback(const sensor_msgs::Imu& msg) {
  // ROS_INFO("imu_callback");
  if (start_imu_) {
    start_imu_ = false;
    old_time_ = msg.header.stamp.toSec();
    return;
  }
  delta_t_ = msg.header.stamp.toSec() - old_time_;
  old_time_ = msg.header.stamp.toSec();

  new_measurement_imu_ = true;
  imu_acc_.x() = msg.linear_acceleration.x;
  imu_acc_.y() = msg.linear_acceleration.y;
  imu_acc_.z() = msg.linear_acceleration.z;

  imu_gyro_.x() = msg.angular_velocity.x;
  imu_gyro_.y() = msg.angular_velocity.y;
  imu_gyro_.z() = msg.angular_velocity.z;
}

bool SensorClient::outlier_detection(Matrix<double, 3, 1> measurement) {
  //	std::cout << "measurement-> " << measurement.transpose()
  //						<< "\nstate-> " <<
  //es_ekf_.getPose().transpose()
  //						<< "\nNorm-> " <<
  //(measurement-es_ekf_.getPose()).norm() << '\n';
  return true;
  return ((measurement - es_ekf_.getP()).norm() < outlier_constant_);
}


void SensorClient::state_estimation(const ros::TimerEvent& msg) {
  if (start_flag_) {
    if (camera_odom_ready()) {
      std::cout << "Rotation matrix\n"
                << params_.camera.rotation_mat << "\n"
                << "Translation " << params_.camera.translation.transpose()
                << "\n"
                << " pose -> " << camera_pose_.translation().transpose() << "\n"
                << " transformed -> " << (this->get_camera_pose()).transpose()
                << '\n';
      es_ekf_.setP(this->get_camera_pose());
      es_ekf_.setQ(this->get_camera_orientation());
      es_ekf_.setV(this->get_camera_lin_vel());
      start_flag_ = false;
    }
    return;
  }
  // PREDICTION
  bool prediction = false;
  bool measurement = false;
  nav_msgs::Odometry ekf_pose_;
  ekf_pose_.pose.covariance.at(0) = std::nan("");
  ekf_pose_.pose.covariance.at(1) = std::nan("");
  ekf_pose_.pose.covariance.at(2) = std::nan("");
  ekf_pose_.twist.twist.angular.x = std::nan("");
  ekf_pose_.twist.twist.angular.y = std::nan("");
  ekf_pose_.twist.twist.angular.z = std::nan("");
  if (start_imu_) {
    ROS_WARN("IMU not ready");
    return;
  }
  if (params_.camera.use_camera_imu && this->camera_imu_ready()) {
    Matrix<double, 3, 3> cam_imu_to_cam = MatrixXd::Zero(3, 3);
    cam_imu_to_cam(0, 2) = 1;
    cam_imu_to_cam(1, 0) = 1;
    cam_imu_to_cam(2, 1) = 1;
    es_ekf_.prediction(cam_imu_to_cam * this->get_acc(), params_.model.Q_f,
                       cam_imu_to_cam * this->get_angular_vel(),
                       params_.model.Q_w, delta_t_);
    prediction = true;
  } else if (new_measurement_imu_) {
//    		es_ekf_.prediction(imu_acc_.translation(),params_.model.Q_f,
//    		imu_gyro_.translation(),params_.model.Q_w,
//    		delta_t_);
    prediction = true;
  }

  if (this->camera_odom_ready()) {
    if (outlier_detection(es_ekf_.getQDrift() * this->get_camera_pose() +
                          es_ekf_.getPDrift())) {
      measurement = true;
      es_ekf_.poseMeasurementUpdateDrift(params_.camera.pose_cov.R,
                                         this->get_camera_pose());
      es_ekf_.angleMeasurementUpdate(MatrixXd::Identity(4, 4) * 0.001,
                                     this->get_camera_orientation_quat());
      camera_state_pub_.publish(true);
      ekf_pose_.pose.covariance.at(0) =
          (es_ekf_.getQDrift() * this->get_camera_pose() +
           es_ekf_.getPDrift())[0];
      ekf_pose_.pose.covariance.at(1) =
          (es_ekf_.getQDrift() * this->get_camera_pose() +
           es_ekf_.getPDrift())[1];
      ekf_pose_.pose.covariance.at(2) =
          (es_ekf_.getQDrift() * this->get_camera_pose() +
           es_ekf_.getPDrift())[2];
    } else {
      camera_state_pub_.publish(false);
      ROS_WARN("Outlier detected in cam measurement");
    }
  }

  if (this->pozyx_ready()) {
    if (outlier_detection(this->get_pozyx_pose())) {
      measurement = true;
      es_ekf_.poseMeasurementUpdate(params_.sensors.at(0).cov.R,
                                    this->get_pozyx_pose());
      ekf_pose_.twist.twist.angular.x = get_pozyx_pose()[0];
      ekf_pose_.twist.twist.angular.y = get_pozyx_pose()[1];
      ekf_pose_.twist.twist.angular.z = get_pozyx_pose()[2];
      pozyx_state_pub_.publish(true);
    } else {
      pozyx_state_pub_.publish(false);
      ROS_WARN("Outlier detected in pozyx measurement");
    }
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
    ekf_pose_.header.stamp = ros::Time::now();
    estimate_pub_.publish(ekf_pose_);
  }
}