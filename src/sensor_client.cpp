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
                              &SensorClient::stateEstimation,
                              this);
}

void SensorClient::stateEstimation(const ros::TimerEvent& /* unused */)
{
  // TODO(lmark): Maybe choose a sensor to initialize EKF (don't initialize it with a
  // random 0th sensor, who knows which one is that?)
  if (m_start_flag) {
    auto sensor_it = std::find_if(
      m_sensor_vector.begin(), m_sensor_vector.end(), [&](const auto& sensor_ptr) {
        return sensor_ptr->getSensorID() == m_ekf_params.initial_sensor_id;
      });

    if (sensor_it == m_sensor_vector.end()) {
      ROS_WARN_STREAM_THROTTLE(
        2.0, "Unable to initialize EKF with sensor: " << m_ekf_params.initial_sensor_id);
      return;
    }

    if ((*sensor_it)->newMeasurement()) {
      m_es_ekf.setP((*sensor_it)->getPose());
      // TODO if orientaion vector
      m_es_ekf.setQ((*sensor_it)->getOrientationVector());
      m_es_ekf.setV({ 0, 0, 0 });
      m_start_flag = false;
      ROS_INFO_STREAM("SensorClient::stateEstimation - EKF initialized with sensor: "
                      << m_ekf_params.initial_sensor_id);
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

    // Get All the measurements
    const auto& sensor_transformed_position = sensor_ptr->getPose();
    const auto& sensor_orientation = sensor_ptr->getOrientation();

    // Call this function after getting all the sensor measurements;
    const auto outlier_checks = sensor_ptr->getOutlierChecks(
      m_es_ekf.getP(), m_es_ekf.getLinVelocity(), m_es_ekf.getOrientation());

    // Update orientation
    if (sensor_ptr->isOrientationSensor() && outlier_checks.orientationValid()) {
      m_es_ekf.angleMeasurementUpdate(sensor_ptr->getROrientation(), sensor_orientation);
      sensor_state += SensorState::ORIENTATION_UPDATE;
    }

    // Update drifted position
    if (sensor_ptr->estimateDrift() && outlier_checks.driftPositionValid()) {
      m_es_ekf.poseMeasurementUpdateDrift(sensor_ptr->getRPose(),
                                          sensor_transformed_position);
      sensor_state += SensorState::POSE_AND_DRIFT_UPDATE;
    }

    // Update regular position
    if (!sensor_ptr->estimateDrift() && outlier_checks.positionValid()) {
      m_es_ekf.poseMeasurementUpdate(sensor_ptr->getRPose(), sensor_transformed_position);
      sensor_state += SensorState::POSE_UPDATE;
    }

    // Update sensor velocity
    if (sensor_ptr->isVelocitySensor() && outlier_checks.linVelocityValid()) {
      // TODO(lmark): Update filter with velocity measurements
      sensor_state += SensorState::LIN_VELOCITY_UPDATE;
    }

    // Publish sensor information
    if (sensor_state > 0) { measurement = true; }
    sensor_ptr->publishState(sensor_state);
    sensor_ptr->publishTransformedPose();
  }

  if (!measurement && !prediction) {
    // TODO(lmark): dead time in state estimation
    return;
  }

  auto        state   = m_es_ekf.getState();
  const auto& pos_cov = m_es_ekf.getPositionCov();
  const auto& vel_cov = m_es_ekf.getLinVelocityCov();
  const auto& q_cov   = m_es_ekf.getOrientationCov();

  nav_msgs::Odometry ekf_pose_;
  ekf_pose_.header.stamp            = ros::Time::now();
  ekf_pose_.header.frame_id         = "world";
  ekf_pose_.pose.pose.position.x    = state[0];
  ekf_pose_.pose.pose.position.y    = state[1];
  ekf_pose_.pose.pose.position.z    = state[2];
  ekf_pose_.twist.twist.linear.x    = state[3];
  ekf_pose_.twist.twist.linear.y    = state[4];
  ekf_pose_.twist.twist.linear.z    = state[5];
  ekf_pose_.pose.pose.orientation.w = state[6];
  ekf_pose_.pose.pose.orientation.x = state[7];
  ekf_pose_.pose.pose.orientation.y = state[8];
  ekf_pose_.pose.pose.orientation.z = state[9];
  ekf_pose_.pose.covariance         = boost::array<double, 36>();
  ekf_pose_.pose.covariance[0]      = pos_cov(0, 0);
  ekf_pose_.pose.covariance[7]      = pos_cov(1, 1);
  ekf_pose_.pose.covariance[14]     = pos_cov(2, 2);
  ekf_pose_.pose.covariance[21]     = q_cov(0, 0);
  ekf_pose_.pose.covariance[27]     = q_cov(1, 1);
  ekf_pose_.pose.covariance[35]     = q_cov(2, 2);
  ekf_pose_.twist.covariance        = boost::array<double, 36>();
  ekf_pose_.twist.covariance[0]     = vel_cov(0, 0);
  ekf_pose_.twist.covariance[7]     = vel_cov(1, 1);
  ekf_pose_.twist.covariance[14]    = vel_cov(2, 2);
  m_estimate_pub.publish(ekf_pose_);
}
