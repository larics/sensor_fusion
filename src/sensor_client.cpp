#include "sensor_client.h"
#include <tf2/LinearMath/Transform.h>

SensorClient::SensorClient(const EsEkfParams& params, ros::NodeHandle& nh_private)
  : SensorClient(params, nh_private, "default")
{}

SensorClient::SensorClient(const EsEkfParams& params,
                           ros::NodeHandle&   nh_private,
                           std::string        uav_name)
  : m_ekf_params(params), m_es_ekf(params), m_start_flag(true),
    m_imu_sensor(params.model, nh_private), m_uav_name(std::move(uav_name)),
    m_sensor_tf(m_uav_name)
{
  // TODO makni flag
  std::string es_ekf_topic;
  nh_private.getParam("es_ekf_topic", es_ekf_topic);
  m_estimate_pub = m_node_handle.advertise<nav_msgs::Odometry>(es_ekf_topic, 1);

  for (const auto& sensor_params : m_ekf_params.sensors) {
    m_sensor_vector.emplace(sensor_params.id, std::make_shared<Sensor>(sensor_params));
  }

  while (ros::ok()) {
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    if (m_imu_sensor.isInit()) {
      bool all_sensor_initialized = true;
      for (const auto& [key, sensor_ptr] : m_sensor_vector) {
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
  if (m_start_flag) {
    const auto& sensor = m_sensor_vector.at(m_ekf_params.initial_sensor_id);

    if (sensor->newMeasurement()) {
      m_es_ekf.setP(sensor->getPose());
      m_es_ekf.setQ(sensor->getOrientation());
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
    auto delta_t = m_imu_sensor.getDeltaT();
    if (delta_t > m_ekf_params.expected_imu_dt) {
      ROS_WARN_THROTTLE(5.0,
                        "[SensorClient] Imu delta t %.2f. Resetting to %.2f",
                        delta_t,
                        m_ekf_params.expected_imu_dt);
      delta_t = m_ekf_params.expected_imu_dt;
    }
    m_es_ekf.prediction(m_imu_sensor.get_acc(),
                        m_ekf_params.model.Q_f,
                        m_imu_sensor.get_angular_vel(),
                        m_ekf_params.model.Q_w,
                        delta_t);
    prediction = true;
  }

  for (const auto& [key, sensor_ptr] : m_sensor_vector) {

    int sensor_state = 0;

    // There are no new measurements
    if (!sensor_ptr->newMeasurement()) { continue; }

    // Get All the measurements
    const auto& sensor_transformed_position = sensor_ptr->getPose();
    const auto& sensor_orientation          = sensor_ptr->getOrientation();

    // Call this function after getting all the sensor measurements;
    const auto outlier_checks = sensor_ptr->getOutlierChecks(
      m_es_ekf.getP(), m_es_ekf.getLinVelocity(), m_es_ekf.getOrientation());

    // Update orientation
    if (sensor_ptr->isOrientationSensor() && outlier_checks.orientationValid()) {
      m_es_ekf.angleMeasurementUpdate(sensor_ptr->getROrientation(), sensor_orientation);
      sensor_state += SensorState::ORIENTATION_UPDATE;
    }

    if (sensor_ptr->isOrientationSensor() && sensor_ptr->estimateDrift()
        && outlier_checks.orientationValid()) {
      m_es_ekf.angleMeasurementUpdateDrift(sensor_ptr->getROrientation(),
                                           sensor_orientation,
                                           sensor_ptr->getTranslationDrift(),
                                           sensor_ptr->getQuaternionDrift(),
                                           sensor_ptr->getDriftPositionCov(),
                                           sensor_ptr->getDriftRotationCov());
      sensor_state += SensorState::ORIENTATION_AND_DRIFT_UPDATE;
    }

    // Update drifted position
    if (sensor_ptr->estimateDrift() && outlier_checks.driftPositionValid()) {
      m_es_ekf.poseMeasurementUpdateDrift(sensor_ptr->getRPose(),
                                          sensor_transformed_position,
                                          sensor_ptr->getTranslationDrift(),
                                          sensor_ptr->getQuaternionDrift(),
                                          sensor_ptr->getDriftPositionCov(),
                                          sensor_ptr->getDriftRotationCov());
      sensor_state += SensorState::POSE_AND_DRIFT_UPDATE;
    }

    // Update regular position
    if (!sensor_ptr->estimateDrift() && outlier_checks.positionValid()) {
      // TODO(lmark): Do the update with sensor_drifted_position
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
    sensor_ptr->publishDrift();
    m_sensor_tf.publishSensorOrigin(
      *sensor_ptr,
      m_es_ekf.getOrientation(), /* Estimated orientation */
      m_sensor_vector.at(m_ekf_params.initial_sensor_id)
        ->getRawOrientation()); /* Main sensor orientation */
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
  m_sensor_tf.publishOrigin(ekf_pose_, "ekf");
}
