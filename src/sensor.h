#ifndef SENSOR_FUSION_SENSOR_H
#define SENSOR_FUSION_SENSOR_H

#include <Eigen/Geometry>
#include <cmath>

#include "nav_msgs/Odometry.h"
#include "ros/forwards.h"
#include "ros/ros.h"
#include "ros/timer.h"
#include "structures.h"
#include "geometry_msgs/TransformStamped.h"
#include "std_msgs/Int32.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace Eigen;
class Sensor;
using SensorPtr = std::shared_ptr<Sensor>;

/*
 * We define sensor subscriber and when an odometry msg comes we
 * update the es-ekf state.
 */

class Sensor
{
private:
  static constexpr auto TIMEOUT = 2;

  ros::Subscriber m_sensor_sub;
  double          m_last_message_time;
  ros::Publisher  m_sensor_drift_pub;
  ros::Publisher  m_sensor_state_pub;
  ros::Publisher  m_transformed_pub;
  ros::Timer      m_watchdog_timer;
  ros::NodeHandle m_node_handle;
  SensorParams    m_sensor_params;
  Translation3d   m_sensor_position;
  Quaterniond     m_sensor_q;
  Quaterniond     m_sensor_transformed_q;
  Vector3d        m_rotated_translation;
  std::string     m_sensor_name;
  bool            m_fresh_position_measurement    = false;
  bool            m_fresh_orientation_measurement = false;
  bool            m_first_measurement             = false;
  bool            m_sensor_responsive             = false;
  Vector3d        m_sensor_transformed_position;
  Translation3d   m_est_position_drift;
  Quaterniond     m_est_quaternion_drift;
  Matrix3d        m_position_drift_cov;
  Matrix3d        m_rotation_drift_cov;

  void update_position()
  {
    m_sensor_transformed_position =
      (m_sensor_params.rotation_mat * m_sensor_position).translation()
      - m_rotated_translation;
  }

  void initialize_sensor_origin(double x, double y, double z)
  {
    ROS_INFO_STREAM(getSensorID() << " origin initialized at first measurement");
    m_sensor_params.translation.x() = x;
    m_sensor_params.translation.y() = y;
    m_sensor_params.translation.z() = z;
    m_rotated_translation = m_sensor_params.rotation_mat * m_sensor_params.translation;
  }

public:
  Sensor(const SensorParams& params)
    : m_sensor_params(params), m_sensor_transformed_position(Vector3d::Zero()),
      m_sensor_name(params.id), m_last_message_time(0)
  {
    if (m_sensor_params.msg_type == SensorMsgType::ODOMETRY) {
      m_sensor_sub = m_node_handle.subscribe(
        m_sensor_params.topic, 1, &Sensor::callbackOdometry, this);
    } else if (m_sensor_params.msg_type == SensorMsgType::TRANSFORM_STAMPED) {
      m_sensor_sub = m_node_handle.subscribe(
        m_sensor_params.topic, 1, &Sensor::callbackTransformStamped, this);
    } else if (m_sensor_params.msg_type == SensorMsgType::POSE_STAMPED) {
      m_sensor_sub = m_node_handle.subscribe(
        m_sensor_params.topic, 1, &Sensor::callbackPoseStamped, this);
    }
    m_sensor_state_pub =
      m_node_handle.advertise<std_msgs::Int32>(m_sensor_params.id + "_state", 1);
    m_transformed_pub = m_node_handle.advertise<geometry_msgs::PoseStamped>(
      m_sensor_params.id + "_transformed_pose", 1);

    if (m_sensor_params.estimate_drift) {
      m_sensor_drift_pub =
        m_node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          m_sensor_params.id + "_drift", 1);
    }

    m_watchdog_timer =
      m_node_handle.createTimer(ros::Duration(TIMEOUT), &Sensor::watchdog_callback, this);
    m_sensor_q             = Quaterniond::Identity();
    m_sensor_transformed_q = Quaterniond::Identity();
    m_rotated_translation  = m_sensor_params.rotation_mat * m_sensor_params.translation;
    m_est_position_drift   = { 0, 0, 0 };
    m_est_quaternion_drift = Quaterniond::Identity();
    m_position_drift_cov   = Matrix3d::Identity() * 0.0001;
    m_rotation_drift_cov   = Matrix3d::Identity() * 0.0001;
  }

  void watchdog_callback(const ros::TimerEvent& /* unused */)
  {
    auto time_diff = ros::Time::now().toSec() - m_last_message_time;
    if (time_diff > TIMEOUT) {
      m_sensor_responsive = false;
      ROS_FATAL("[Sensor] %s is unresponsive!", m_sensor_name.c_str());
    } else {
      m_sensor_responsive = true;
    }
  }

  void publishState(int state)
  {
    std_msgs::Int32 state_msg;
    state_msg.data = state;
    m_sensor_state_pub.publish(state_msg);
  }

  void publishTransformedPose()
  {
    geometry_msgs::PoseStamped transformed_msg;
    Vector3d                   position    = m_sensor_transformed_position;
    Quaterniond                orientation = m_sensor_transformed_q;

    if (estimateDrift()) {
      position    = getDriftedPose();
      orientation = getDriftedRotation();
    }
    transformed_msg.header.frame_id    = "world";
    transformed_msg.header.stamp       = ros::Time::now();
    transformed_msg.pose.position.x    = m_sensor_transformed_position.x();
    transformed_msg.pose.position.y    = m_sensor_transformed_position.y();
    transformed_msg.pose.position.z    = m_sensor_transformed_position.z();
    transformed_msg.pose.orientation.x = m_sensor_transformed_q.x();
    transformed_msg.pose.orientation.y = m_sensor_transformed_q.y();
    transformed_msg.pose.orientation.z = m_sensor_transformed_q.z();
    transformed_msg.pose.orientation.w = m_sensor_transformed_q.w();
    m_transformed_pub.publish(transformed_msg);
  }

  void publishDrift()
  {
    if (!m_sensor_params.estimate_drift) { return; }
    geometry_msgs::PoseWithCovarianceStamped drift_pose;
    drift_pose.header.frame_id         = "world";
    drift_pose.header.stamp            = ros::Time::now();
    drift_pose.pose.pose.position.x    = m_est_position_drift.x();
    drift_pose.pose.pose.position.y    = m_est_position_drift.y();
    drift_pose.pose.pose.position.z    = m_est_position_drift.z();
    drift_pose.pose.pose.orientation.x = m_est_quaternion_drift.x();
    drift_pose.pose.pose.orientation.y = m_est_quaternion_drift.y();
    drift_pose.pose.pose.orientation.z = m_est_quaternion_drift.z();
    drift_pose.pose.pose.orientation.w = m_est_quaternion_drift.w();
    drift_pose.pose.covariance         = boost::array<double, 36>();
    drift_pose.pose.covariance[0]      = m_position_drift_cov(0, 0);
    drift_pose.pose.covariance[7]      = m_position_drift_cov(1, 1);
    drift_pose.pose.covariance[14]     = m_position_drift_cov(2, 2);
    drift_pose.pose.covariance[21]     = m_rotation_drift_cov(0, 0);
    drift_pose.pose.covariance[27]     = m_rotation_drift_cov(1, 1);
    drift_pose.pose.covariance[35]     = m_rotation_drift_cov(2, 2);
    m_sensor_drift_pub.publish(drift_pose);
  }

  void callbackPoseStamped(const geometry_msgs::PoseStamped& msg)
  {
    if (!std::isfinite(msg.pose.position.x) || !std::isfinite(msg.pose.position.y)
        || !std::isfinite(msg.pose.position.z) || !std::isfinite(msg.pose.orientation.x)
        || !std::isfinite(msg.pose.orientation.y)
        || !std::isfinite(msg.pose.orientation.z)
        || !std::isfinite(msg.pose.orientation.w)) {
      ROS_FATAL("[Sensor] %s returned invalid measurement.", m_sensor_name.c_str());
      return;
    }

    // ROS_INFO("camera_posix_callback");
    if (!m_first_measurement && m_sensor_params.origin_at_first_measurement) {
      m_first_measurement = true;
      initialize_sensor_origin(
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    }

    m_fresh_position_measurement = true;
    m_sensor_position.x()        = msg.pose.position.x;
    m_sensor_position.y()        = msg.pose.position.y;
    m_sensor_position.z()        = msg.pose.position.z;

    m_fresh_orientation_measurement = true;
    m_sensor_q.w()                  = msg.pose.orientation.w;
    m_sensor_q.x()                  = msg.pose.orientation.x;
    m_sensor_q.y()                  = msg.pose.orientation.y;
    m_sensor_q.z()                  = msg.pose.orientation.z;

    m_last_message_time = ros::Time::now().toSec();
  }

  void callbackTransformStamped(const geometry_msgs::TransformStamped& msg)
  {
    if (!std::isfinite(msg.transform.translation.x)
        || !std::isfinite(msg.transform.translation.y)
        || !std::isfinite(msg.transform.translation.z)
        || !std::isfinite(msg.transform.rotation.x)
        || !std::isfinite(msg.transform.rotation.y)
        || !std::isfinite(msg.transform.rotation.z)
        || !std::isfinite(msg.transform.rotation.w)) {
      ROS_FATAL("[Sensor] %s returned invalid measurement.", m_sensor_name.c_str());
      return;
    }

    // ROS_INFO("camera_posix_callback");
    if (!m_first_measurement && m_sensor_params.origin_at_first_measurement) {
      m_first_measurement = true;
      initialize_sensor_origin(msg.transform.translation.x,
                               msg.transform.translation.y,
                               msg.transform.translation.z);
    }

    m_fresh_position_measurement = true;
    m_sensor_position.x()        = msg.transform.translation.x;
    m_sensor_position.y()        = msg.transform.translation.y;
    m_sensor_position.z()        = msg.transform.translation.z;

    m_fresh_orientation_measurement = true;
    m_sensor_q.w()                  = msg.transform.rotation.w;
    m_sensor_q.x()                  = msg.transform.rotation.x;
    m_sensor_q.y()                  = msg.transform.rotation.y;
    m_sensor_q.z()                  = msg.transform.rotation.z;

    m_last_message_time = ros::Time::now().toSec();
  }

  void callbackOdometry(const nav_msgs::OdometryPtr& msg)
  {
    if (!std::isfinite(msg->pose.pose.position.x)
        || !std::isfinite(msg->pose.pose.position.y)
        || !std::isfinite(msg->pose.pose.position.z)
        || !std::isfinite(msg->pose.pose.orientation.x)
        || !std::isfinite(msg->pose.pose.orientation.y)
        || !std::isfinite(msg->pose.pose.orientation.z)
        || !std::isfinite(msg->pose.pose.orientation.w)) {
      ROS_FATAL("[Sensor] %s returned invalid measurement.", m_sensor_name.c_str());
      return;
    }

    if (!m_first_measurement && m_sensor_params.origin_at_first_measurement) {
      m_first_measurement = true;
      initialize_sensor_origin(
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }

    m_fresh_position_measurement = true;
    m_sensor_position.x()        = msg->pose.pose.position.x;
    m_sensor_position.y()        = msg->pose.pose.position.y;
    m_sensor_position.z()        = msg->pose.pose.position.z;

    m_fresh_orientation_measurement = true;
    m_sensor_q.w()                  = msg->pose.pose.orientation.w;
    m_sensor_q.x()                  = msg->pose.pose.orientation.x;
    m_sensor_q.y()                  = msg->pose.pose.orientation.y;
    m_sensor_q.z()                  = msg->pose.pose.orientation.z;

    m_last_message_time = ros::Time::now().toSec();
  }

  const Vector3d& getPose()
  {
    if (m_fresh_position_measurement) {
      m_fresh_position_measurement = false;
      update_position();
    }
    return m_sensor_transformed_position;
  }

  const Translation3d& getRawPosition() const { return m_sensor_position; }

  const Quaterniond& getRawOrientation() const { return m_sensor_q; }

  const Quaterniond& getOrientation()
  {
    if (isOrientationSensor() && m_fresh_orientation_measurement) {
      m_fresh_position_measurement = false;
      m_sensor_transformed_q       = m_sensor_params.rotation_mat * m_sensor_q;
    }
    return m_sensor_transformed_q;
  }

  OutlierChecks getOutlierChecks(const Vector3d&    position,
                                 const Vector3d&    lin_vel,
                                 const Quaterniond& rotation)
  {
    OutlierChecks checks;

    // Orientation checks
    if (isOrientationSensor()) {
      // TODO(lmark): Calculate orientation checks
      checks.orientation_outlier = false;
      if (checks.orientation_outlier) {
        ROS_WARN_STREAM_THROTTLE(
          2.0,
          "Sensor::getOutlierChecks - orientation outlier check failed for sensor: "
            << getSensorID());
      }
    }

    // Drift position checks
    if (estimateDrift()) {
      // TODO(lmark): Add drifted position outlier
      const auto& drifted_pos = getDriftedPose();
      auto        abs_error   = (position - drifted_pos).cwiseAbs();
      checks.drifted_position_outlier =
        abs_error.x() > m_sensor_params.position_outlier_lim.x()
        || abs_error.y() > m_sensor_params.position_outlier_lim.y()
        || abs_error.z() > m_sensor_params.position_outlier_lim.z();

      if (checks.drifted_position_outlier) {
        ROS_WARN_THROTTLE(
          2.0,
          "Sensor::getOutlierChecks - drifted position outlier check [%.2f, %.2f, "
          "%.2f]-[%.2f, %.2f, %.2f]=[%.2f, %.2f, %.2f] failed for sensor: %s",
          position.x(),
          position.y(),
          position.z(),
          drifted_pos.x(),
          drifted_pos.y(),
          drifted_pos.z(),
          abs_error.x(),
          abs_error.y(),
          abs_error.z(),
          getSensorID().c_str());
      }
    }

    // Regular position checks
    if (!estimateDrift()) {
      auto abs_error = (position - m_sensor_transformed_position).cwiseAbs();
      checks.position_outlier =
        abs_error.x() > m_sensor_params.position_outlier_lim.x()
        || abs_error.y() > m_sensor_params.position_outlier_lim.y()
        || abs_error.z() > m_sensor_params.position_outlier_lim.z();

      if (checks.position_outlier) {
        ROS_WARN_THROTTLE(
          2.0,
          "Sensor::getOutlierChecks - position outlier check [%.2f, %.2f, "
          "%.2f]-[%.2f, %.2f, %.2f]=[%.2f, %.2f, %.2f] failed for sensor: %s",
          position.x(),
          position.y(),
          position.z(),
          m_sensor_transformed_position.x(),
          m_sensor_transformed_position.y(),
          m_sensor_transformed_position.z(),
          abs_error.x(),
          abs_error.y(),
          abs_error.z(),
          getSensorID().c_str());
      }
    }

    if (isVelocitySensor()) {
      // TODO(lmark): Calculate velocity checks
      checks.lin_velocity_outlier = false;
      if (checks.lin_velocity_outlier) {
        ROS_WARN_STREAM_THROTTLE(
          2.0,
          "Sensor::getOutlierChecks - linear velocity outlier check failed for sensor: "
            << getSensorID());
      }
    }

    return checks;
  }

  Vector3d getDriftedPose() const
  {
    return m_est_quaternion_drift.inverse() * (m_sensor_transformed_position);
  }
  Quaterniond getDriftedRotation() const
  {
    return m_est_quaternion_drift.inverse() * m_sensor_transformed_q;
  }
  bool               isResponsive() const { return m_sensor_responsive; }
  const std::string& getName() const { return m_sensor_name; }
  const Vector3d&    getPose() const { return m_sensor_transformed_position; }
  const Quaterniond& getOrientation() const { return m_sensor_transformed_q; }
  void               setR(Matrix3d R) { m_sensor_params.cov.R_pose = std::move(R); }
  bool               newMeasurement() const { return m_fresh_position_measurement; }
  bool isOrientationSensor() const { return m_sensor_params.is_orientation_sensor; }
  bool isVelocitySensor() const { return false; }
  bool estimateDrift() const { return m_sensor_params.estimate_drift; }
  const std::string& getSensorID() const { return m_sensor_params.id; }
  const Matrix3d&    getRPose() const { return m_sensor_params.cov.R_pose; }
  const Matrix3d&    getROrientation() const { return m_sensor_params.cov.R_orientation; }
  Quaterniond&       getQuaternionDrift() { return m_est_quaternion_drift; }
  Vector3d&          getTranslationDrift() { return m_est_position_drift.vector(); }
  Matrix3d&          getDriftPositionCov() { return m_position_drift_cov; }
  Matrix3d&          getDriftRotationCov() { return m_rotation_drift_cov; }
  ~Sensor() { std::cout << "SENSOR DESTRUCTOR" << '\n'; }
};
#endif
