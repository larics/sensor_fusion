#include "sensor_client.h"

SensorClient::SensorClient(const EsEkfParams& params, ros::NodeHandle& nh_private)
  : m_ekf_params(params), m_es_ekf(params), m_start_flag(true),
    m_imu_sensor(params.model, nh_private)
{
  // TODO makni flag
  std::string es_ekf_topic;
  nh_private.getParam("es_ekf_topic", es_ekf_topic);
  m_estimate_pub = m_node_handle.advertise<nav_msgs::Odometry>(es_ekf_topic, 1);

  for (const auto& sensor_params : m_ekf_params.sensors) {
    m_sensor_vector.push_back(std::make_shared<Sensor>(sensor_params));
  }

  while (ros::ok()) {
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    if (m_imu_sensor.isInit()) {
      bool all_sensor_initialized = true;
      for (const auto& sensor_ptr : m_sensor_vector) {
        if (sensor_ptr->newMeasurement()) {
          ROS_INFO_STREAM("SensorClient::SensorClient() - Sensor "
                          << sensor_ptr->getSensorID() << " initilized.");
        } else {
          ROS_WARN_STREAM("SensorClient::SensorClient() - Unable to initialize "
                          << sensor_ptr->getSensorID() << " sensor.");
          all_sensor_initialized = false;
        }
      }

      if (all_sensor_initialized) {
        ROS_INFO("SensorClient::SensorClient() - All Sensors Initialized!");
        break;
      }

    } else {
      ROS_WARN("Now we wait for sensors...");
      if (ros::isShuttingDown()) { break; }
    }
  }
  
  m_update_timer =
    m_node_handle.createTimer(ros::Duration(1. / m_ekf_params.estimation_frequncy),
                              &SensorClient::state_estimation,
                              this);
}

bool SensorClient::outlier_detection(const Matrix<double, 3, 1>& measurement)
{
  // TODO(lmark): Move outlier detection to sensor.h
  return ((measurement - m_es_ekf.getP()).norm() < m_ekf_params.outlier_constant);
}

void SensorClient::state_estimation(const ros::TimerEvent& msg)
{
  // TODO(lmark): Maybe choose a sensor to initialize EKF (don't initialize it with a
  // random 0th sensor, who knows which one is that?)
  if (m_start_flag) {
    if (m_sensor_vector.at(0)->newMeasurement()) {
      m_es_ekf.setP(m_sensor_vector.at(0)->getPose());
      // TODO if orientaion vector
      m_es_ekf.setQ(m_sensor_vector.at(0)->getOrientationVector());
      m_es_ekf.setV({ 0, 0, 0 });
      m_start_flag = false;
    }
    return;
  }

  // PREDICTION
  bool prediction  = false;
  bool measurement = false;
  // TODO ovo makni, provjerava se u inicijalizaciji
  if (!m_imu_sensor.isInit()) {
    ROS_WARN_THROTTLE(5.0, "IMU not ready");
    return;
  }
  if (m_imu_sensor.newMeasurement()) {
    m_es_ekf.prediction(m_imu_sensor.get_acc(),
                        m_ekf_params.model.Q_f,
                        m_imu_sensor.get_angular_vel(),
                        m_ekf_params.model.Q_w,
                        m_imu_sensor.getDeltaT());
    prediction = true;
  }

  for (const auto& sensor_ptr : m_sensor_vector) {

    int sensor_state = 0;

    // There are no new measurements
    if (!sensor_ptr->newMeasurement()) { continue; }

    // Update orientation
    // TODO(lmark): Add outlier checking for orientation
    if (sensor_ptr->isOrientationSensor()) {
      m_es_ekf.angleMeasurementUpdate(sensor_ptr->getROrientation(),
                                      sensor_ptr->getOrientation());
      sensor_state += SensorState::ORIENTATION_UPDATE;
    }

    if (sensor_ptr->estimateDrift()
        && outlier_detection(
          sensor_ptr->getDriftedPose(m_es_ekf.getQDrift(), m_es_ekf.getPDrift()))) {
      // If the sensor should estimate drift - Update drift and position
      m_es_ekf.poseMeasurementUpdateDrift(sensor_ptr->getRPose(), sensor_ptr->getPose());
      sensor_state += SensorState::POSE_AND_DRIFT_UPDATE;

    } else if (outlier_detection(sensor_ptr->getPose())) {
      // Otherwise sensor does a regular measurement update
      m_es_ekf.poseMeasurementUpdate(sensor_ptr->getRPose(), sensor_ptr->getPose());
      sensor_state += SensorState::POSE_UPDATE;
    }

    if (sensor_state > 0) { measurement = true; }
    sensor_ptr->publishState(sensor_state);
    sensor_ptr->publishTransformedPose();
  }

  if (measurement or prediction) {
    Matrix<double, 10, 1> state;
    state = m_es_ekf.getState();

    nav_msgs::Odometry ekf_pose_;
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
    m_estimate_pub.publish(ekf_pose_);
  }
}
