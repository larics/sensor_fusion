#include "sensor_client.h"

SensorClient::SensorClient(const EsEkfParams& params, ros::NodeHandle& nh_private)
  : m_ekf_params(params), m_es_ekf(params), m_start_flag(true),
    m_imu_sensor(params.model, nh_private)
{
  // TODO makni flag
  std::string es_ekf_topic;
  nh_private.getParam("es_ekf_topic", es_ekf_topic);
  m_estimate_pub = m_node_handle.advertise<nav_msgs::Odometry>(es_ekf_topic, 1);

  for (int i = 0; i < m_ekf_params.sensors.size(); ++i) {
    m_sensor_vector.push_back(new Sensor(m_ekf_params.sensors.at(i)));
  }
  while (ros::ok()) {
    ros::Duration(0.25).sleep();
    ros::spinOnce();
    if (m_imu_sensor.isInit()) {
      int k = 0;
      for (int i = 0; i < m_ekf_params.sensors.size(); ++i) {
        if (m_sensor_vector.at(i)->newMeasurement()) { k++; }
      }
      if (k == m_ekf_params.sensors.size()) {
        ROS_INFO_STREAM("SensorClient::SensorClient() - Sensor Initialized!");
        break;
      }

      ROS_INFO_STREAM("Waiting for all sensors. Initialized: ["
                      << k << "/" << m_ekf_params.sensors.size() << "]");

    } else {
      ROS_WARN("Now we wait for sensors...");
      if (ros::isShuttingDown()) break;
    }
  }
  // TODO izvuci van
  m_update_timer =
    m_node_handle.createTimer(ros::Duration(0.01), &SensorClient::state_estimation, this);
}

bool SensorClient::outlier_detection(const Matrix<double, 3, 1>& measurement)
{
  return ((measurement - m_es_ekf.getP()).norm() < m_ekf_params.outlier_constant);
}

void SensorClient::state_estimation(const ros::TimerEvent& msg)
{
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
  bool               prediction  = false;
  bool               measurement = false;
  nav_msgs::Odometry ekf_pose_;
  // TODO ovo makni, provjerava se u inicijalizaciji
  if (!m_imu_sensor.isInit()) {
    ROS_WARN_THROTTLE(5.0, "IMU not ready");
    return;
  }
  if (m_imu_sensor.newMeasurement()) {
    // std::cout << "Linear acc -> " << m_imu_sensor.get_acc();
    m_es_ekf.prediction(m_imu_sensor.get_acc(),
                        m_ekf_params.model.Q_f,
                        m_imu_sensor.get_angular_vel(),
                        m_ekf_params.model.Q_w,
                        m_imu_sensor.getDeltaT());
    prediction = true;
  }

  for (int i = 0; i < m_sensor_vector.size(); ++i) {
    if (m_sensor_vector.at(i)->newMeasurement()) {

      if (m_sensor_vector.at(i)->estimateDrift()
          && outlier_detection(m_sensor_vector.at(i)->getDriftedPose(
            m_es_ekf.getQDrift(), m_es_ekf.getPDrift()))) {
        measurement = true;
        m_es_ekf.poseMeasurementUpdateDrift(m_sensor_vector.at(i)->getRPose(),
                                            m_sensor_vector.at(i)->getPose());

        if (m_sensor_vector.at(i)->isOrientationSensor()) {
          m_es_ekf.angleMeasurementUpdate(m_sensor_vector.at(i)->getROrientation(),
                                          m_sensor_vector.at(i)->getOrientation());
        }
        m_sensor_vector.at(i)->publishState(true);

      } else if (outlier_detection(m_sensor_vector.at(i)->getPose())) {
        measurement = true;
        m_es_ekf.poseMeasurementUpdate(m_sensor_vector.at(i)->getRPose(),
                                       m_sensor_vector.at(i)->getPose());
        m_sensor_vector.at(i)->publishState(true);
      } else {
        m_sensor_vector.at(i)->publishState(false);
      }
    }

    m_sensor_vector.at(i)->publishTransformedPose();
  }

  if (measurement or prediction) {
    Matrix<double, 10, 1> state;
    state = m_es_ekf.getState();

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
