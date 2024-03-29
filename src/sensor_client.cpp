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
    m_sensor_tf(m_uav_name), m_tf_listener(m_tf_buffer)
{
  // TODO makni flag
  std::string es_ekf_topic;
  nh_private.getParam("es_ekf_topic", es_ekf_topic);

  nh_private.getParam("odom_helper_enable", this->m_odom_helper_enable);
  ROS_WARN_COND(this->m_odom_helper_enable, "[SensorClient] Odom helper topic enabled");

  // Get params for multiframe publishers
  nh_private.getParam("base_frame", m_base_frame);
  nh_private.getParam("odom_topics", m_odom_topics);
  nh_private.getParam("frame_ids", m_frame_ids);
  if (m_odom_topics.size() != m_frame_ids.size()) {
    ROS_FATAL("SensorClient::SensorClient() odom_topics is not the same as frame_ids. Throwing.");
    throw;
  }
  for (auto topic : m_odom_topics) {
    ROS_INFO_STREAM("Adding topic " << topic << " to list of publishers.");
    ros::Publisher topic_pub;
    topic_pub = m_node_handle.advertise<nav_msgs::Odometry>(topic, 1);
    m_estimate_pubs.push_back(topic_pub);
  }
  ROS_INFO_STREAM("Finished adding additional frames to the list of publishers.\n");

  m_estimate_pub = m_node_handle.advertise<nav_msgs::Odometry>(es_ekf_topic, 1);
  m_helper_odom_sub =
    m_node_handle.subscribe("odom_helper_topic", 1, &SensorClient::helper_odom_cb, this);

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
    if (!sensor_ptr->newMeasurement()) {

      // If there are no new sensor measurements
      if (!sensor_ptr->isResponsive()) { sensor_ptr->publishState(0); }
      continue;
    }

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
    if (sensor_ptr->isPositionSensor() && sensor_ptr->estimateDrift()
        && outlier_checks.driftPositionValid()) {
      m_es_ekf.poseMeasurementUpdateDrift(sensor_ptr->getRPose(),
                                          sensor_transformed_position,
                                          sensor_ptr->getTranslationDrift(),
                                          sensor_ptr->getQuaternionDrift(),
                                          sensor_ptr->getDriftPositionCov(),
                                          sensor_ptr->getDriftRotationCov());
      sensor_state += SensorState::POSE_AND_DRIFT_UPDATE;
    }

    // Update regular position
    if (sensor_ptr->isPositionSensor() && !sensor_ptr->estimateDrift()
        && outlier_checks.positionValid()) {
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

  if (m_odom_helper_enable) {
    ROS_INFO_THROTTLE(2.0, "[SensorClient] - Helper enabled");
    ekf_pose_.pose.pose.position.z = m_helper_odom.pose.pose.position.z;
    ekf_pose_.twist.twist.linear.z = -m_helper_odom.twist.twist.linear.z;
    ekf_pose_.twist.twist.linear.y = 0;
    ekf_pose_.twist.twist.linear.x = 0;
  }

  m_estimate_pub.publish(ekf_pose_);
  m_sensor_tf.publishOrigin(ekf_pose_, "ekf");

  this->transformAndPublishInFrames(ekf_pose_);
  
}

void SensorClient::helper_odom_cb(const nav_msgs::Odometry& msg) { m_helper_odom = msg; }

inline void SensorClient::transformAndPublishInFrames(const nav_msgs::Odometry& ekf_pose_)
{
  for (int i=0; i<m_frame_ids.size(); i++) {
    // Transform and publish additional sensors
    geometry_msgs::TransformStamped transform_stamped;
    try {
      transform_stamped = m_tf_buffer.lookupTransform(m_frame_ids[i], m_base_frame,
                                  ros::Time(0));
      m_estimate_pubs[i].publish(transformOdom(ekf_pose_, transform_stamped));
    }
    catch (tf2::TransformException &ex) {
      // Warn user if the transform does not exist
      ROS_WARN_THROTTLE(5.0, "SensorClient::transformAndPublishInFrames() Could not transform between frames: %s",
        ex.what());
      continue;
    }
  }
}

nav_msgs::Odometry transformOdom(const nav_msgs::Odometry& odom_in,
  const geometry_msgs::TransformStamped& transform)
{
  nav_msgs::Odometry odom_out;
  odom_out.header.stamp = odom_in.header.stamp;
  // Set up the transform as affine
  Eigen::Affine3d transform_affine;
  transform_affine = Eigen::Affine3d::Identity();
  Eigen::Quaterniond transform_q;
  transform_q.x() = transform.transform.rotation.x;
  transform_q.y() = transform.transform.rotation.y;
  transform_q.z() = transform.transform.rotation.z;
  transform_q.w() = transform.transform.rotation.w;
  transform_affine.translate(Eigen::Vector3d(transform.transform.translation.x,
    transform.transform.translation.y,
    transform.transform.translation.z));
  transform_affine.rotate(transform_q.normalized().toRotationMatrix());

  // Put odometry into eigen stuff
  Eigen::Affine3d odom_in_affine;
  odom_in_affine = Eigen::Affine3d::Identity();
  Eigen::Quaterniond odom_in_q;
  odom_in_q.x() = odom_in.pose.pose.orientation.x;
  odom_in_q.y() = odom_in.pose.pose.orientation.y;
  odom_in_q.z() = odom_in.pose.pose.orientation.z;
  odom_in_q.w() = odom_in.pose.pose.orientation.w;
  odom_in_affine.translate(Eigen::Vector3d(odom_in.pose.pose.position.x,
    odom_in.pose.pose.position.y,
    odom_in.pose.pose.position.z));
  odom_in_affine.rotate(odom_in_q.normalized().toRotationMatrix());
  // Velocities
  Eigen::Vector3d odom_in_linear_velocity;
  odom_in_linear_velocity(0) = odom_in.twist.twist.linear.x;
  odom_in_linear_velocity(1) = odom_in.twist.twist.linear.y;
  odom_in_linear_velocity(2) = odom_in.twist.twist.linear.z;
  Eigen::Vector3d odom_in_angular_velocity;
  odom_in_angular_velocity(0) = odom_in.twist.twist.angular.x;
  odom_in_angular_velocity(1) = odom_in.twist.twist.angular.y;
  odom_in_angular_velocity(2) = odom_in.twist.twist.angular.z;
  
  // Transform everything
  // TODO: transform covariance. I think you just put two 3x3 rotation matrices
  // diagonally into 6x6 and transform the 6x6 covariance matrix.
  // Cov_out = R*Cov_in*R_transpose
  Eigen::Affine3d odom_out_affine;
  odom_out_affine = transform_affine*odom_in_affine;
  Eigen::Vector3d odom_out_linear_velocity;
  odom_out_linear_velocity = transform_affine.matrix().block<3,3>(0,0)*odom_in_linear_velocity;
  Eigen::Vector3d odom_out_angular_velocity;
  odom_out_angular_velocity = transform_affine.matrix().block<3,3>(0,0)*odom_in_angular_velocity;

  // Put everything back into odometry
  odom_out.pose.pose.position.x = odom_out_affine.translation().x();
  odom_out.pose.pose.position.y = odom_out_affine.translation().y();
  odom_out.pose.pose.position.z = odom_out_affine.translation().z();
  Eigen::Quaterniond q;
  q = odom_out_affine.matrix().block<3,3>(0,0);
  odom_out.pose.pose.orientation.x = q.x();
  odom_out.pose.pose.orientation.y = q.y();
  odom_out.pose.pose.orientation.z = q.z();
  odom_out.pose.pose.orientation.w = q.w();
  odom_out.twist.twist.linear.x = odom_out_linear_velocity(0);
  odom_out.twist.twist.linear.y = odom_out_linear_velocity(1);
  odom_out.twist.twist.linear.z = odom_out_linear_velocity(2);
  odom_out.twist.twist.angular.x = odom_out_angular_velocity(0);
  odom_out.twist.twist.angular.y = odom_out_angular_velocity(1);
  odom_out.twist.twist.angular.z = odom_out_angular_velocity(2);

  return odom_out;
}