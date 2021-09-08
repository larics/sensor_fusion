//
// Created by kovac on 05. 12. 2020..
//

#include "filter.h"

EsEkf2::EsEkf2(const EsEkfParams& params)
{

  // motion model noise jacobian
  m_jacobian = MatrixXd::Zero(N_STATES, N_STATES_2);
  m_jacobian.block<N_STATES_2, N_STATES_2>(3, 0) =
    MatrixXd::Identity(N_STATES_2, N_STATES_2);

  // initial state for pose, linear velocity and orientation
  m_est_position       = { 0, 0, 0 };
  m_est_lin_velocity   = { 0, 0, 0 };
  m_est_quaternion     = { 1, 0, 0, 0 };
  m_est_acc_bias       = { 0, 0, 0 };
  m_acc_bias_variance  = params.acc_bias_variance;
  m_gyro_bias_variance = params.gyro_bias_variance;
  m_est_gyro_bias      = { 0, 0, 0 };
  m_est_gravity        = params.g.vector();

  // initital state error is zero (can be anything else),18 nuber of core states
  // p,v,q,ab,wb,g
  m_p_covariance = IdentityNxN;
  m_p_covariance.block<3, 3>(POSITION_IDX, POSITION_IDX) =
    params.init_pose_p_cov * Identity3x3;
  m_p_covariance.block<3, 3>(VELOCITY_IDX, VELOCITY_IDX) =
    params.init_v_p_cov * Identity3x3;
  m_p_covariance.block<3, 3>(ANGLE_AXIS_IDX, ANGLE_AXIS_IDX) =
    params.init_q_p_cov * Identity3x3;
  m_p_covariance.block<3, 3>(ACC_BIAS_IDX, ACC_BIAS_IDX) =
    params.init_ab_p_cov * Identity3x3;
  m_p_covariance.block<3, 3>(GYRO_BIAS_IDX, GYRO_BIAS_IDX) =
    params.init_wb_p_cov * Identity3x3;
  m_p_covariance.block<3, 3>(GRAVITY_IDX, GRAVITY_IDX) = params.init_g_cov * Identity3x3;
  m_p_covariance.block<3, 3>(DRIFT_TRANS_IDX, DRIFT_TRANS_IDX) =
    params.init_p_drift * Identity3x3;
  m_p_covariance.block<3, 3>(DRIFT_ROT_IDX, DRIFT_ROT_IDX) =
    params.init_q_drift * Identity3x3;

  if (!params.estimate_acc_bias) {
    m_p_covariance.block<3, 3>(ACC_BIAS_IDX, ACC_BIAS_IDX) = MatrixXd::Zero(3, 3);
    m_acc_bias_variance                                    = Matrix3d::Zero();
  }
  if (!params.estimate_gyro_bias) {
    m_p_covariance.block<3, 3>(GYRO_BIAS_IDX, GYRO_BIAS_IDX) = MatrixXd::Zero(3, 3);
    m_gyro_bias_variance                                     = Matrix3d::Zero();
  }
  if (!params.estimate_gravity) {
    m_p_covariance.block<3, 3>(GRAVITY_IDX, GRAVITY_IDX) = MatrixXd::Zero(3, 3);
  }
  std::cout << "m_jacobianobian:\n"
            << m_jacobian << '\n'
            << "m_p_covariance init:\n"
            << m_p_covariance << '\n';
}

void EsEkf2::prediction(const Matrix<double, 3, 1>& imu_f,
                        const Matrix3d&             var_imu_f,
                        const Matrix<double, 3, 1>& imu_w,
                        const Matrix3d&             var_imu_w,
                        const double                delta_t)
{

  // TODO transform imu maybe beore prediction????
  // 1.1 Linearize the motion model
  // and compute Jacobians (Scola (311))
  Quaterniond delta_q;
  auto        delta_theta = delta_t * (imu_w - m_est_gyro_bias);
  delta_q                 = AngleAxisd(delta_theta.norm(), delta_theta.normalized());
  auto                               rot_est       = m_est_quaternion.toRotationMatrix();
  Matrix<double, N_STATES, N_STATES> f_jac         = IdentityNxN;
  f_jac.block<3, 3>(POSITION_IDX, VELOCITY_IDX)    = delta_t * Identity3x3;
  f_jac.block<3, 3>(VELOCITY_IDX, ACC_BIAS_IDX)    = -delta_t * rot_est;
  f_jac.block<3, 3>(VELOCITY_IDX, GRAVITY_IDX)     = delta_t * Identity3x3;
  f_jac.block<3, 3>(ANGLE_AXIS_IDX, GYRO_BIAS_IDX) = -delta_t * Identity3x3;
  f_jac.block<3, 3>(VELOCITY_IDX, ANGLE_AXIS_IDX) =
    -rot_est * sf::skew_symetric(imu_f - m_est_acc_bias) * delta_t;
  f_jac.block<3, 3>(ANGLE_AXIS_IDX, ANGLE_AXIS_IDX) =
    delta_q.toRotationMatrix().transpose();
  f_jac.block<6, 6>(DRIFT_TRANS_IDX, DRIFT_TRANS_IDX) = Matrix<double, 6, 6>::Zero();

  // 2. Propagate uncertainty
  // Components on pg. 60 (262-265)
  // Definition on pg. 70 (312)
  // TODO(lmark): var_imu_f, var_imu_w, m_acc_bias_variance, m_gyro_bias_variance should be
  // renamed to velocity varniance, orientation variance, acceleration bias variance,
  // angular velocity bias variance
  
  // TODO(lmark): We should probably add sensor position and orientation drift variance
  Matrix<double, 12, 12> q_cov = Matrix<double, 12, 12>::Identity();
  q_cov.block<3, 3>(0, 0)      = var_imu_f * pow(delta_t, 2);
  q_cov.block<3, 3>(3, 3)      = var_imu_w * pow(delta_t, 2);
  q_cov.block<3, 3>(6, 6)      = m_acc_bias_variance * delta_t;
  q_cov.block<3, 3>(9, 9)      = m_gyro_bias_variance * delta_t;
  m_p_covariance               = f_jac * m_p_covariance * f_jac.transpose()
                   + m_jacobian * q_cov * m_jacobian.transpose();

  // Update nominal state with IMU inputs
  // Use current 'rot_est' not the new updated one
  m_est_position +=
    +delta_t * m_est_lin_velocity
    + pow(delta_t, 2) / 2.0 * (rot_est * (imu_f - m_est_acc_bias) + m_est_gravity);

  m_est_lin_velocity += +delta_t * (rot_est * (imu_f - m_est_acc_bias) + (m_est_gravity));

  m_est_quaternion = m_est_quaternion * delta_q;
  m_est_quaternion.normalize();

  ROS_INFO_THROTTLE(2.0, "EsEkf2::prediction()");
}

void EsEkf2::inject_error_state(const Matrix<double, N_STATES, 1>& error_state)
{
  // 3.3 Correct predicted state
  m_est_position += error_state.block<3, 1>(POSITION_IDX, 0);
  m_est_lin_velocity += error_state.block<3, 1>(VELOCITY_IDX, 0);

  const auto& angle_axis_vector = error_state.block<3, 1>(ANGLE_AXIS_IDX, 0);
  Quaterniond delta_q;
  delta_q = Eigen::AngleAxisd(angle_axis_vector.norm(), angle_axis_vector.normalized());

  // pg. 68 (303)
  m_est_quaternion = m_est_quaternion * delta_q;
  m_est_quaternion.normalize();

  m_est_acc_bias += error_state.block<3, 1>(ACC_BIAS_IDX, 0);
  m_est_gyro_bias += error_state.block<3, 1>(GYRO_BIAS_IDX, 0);
  m_est_gravity += error_state.block<3, 1>(GRAVITY_IDX, 0);
}

void EsEkf2::inject_error_state_drift(const Matrix<double, N_STATES, 1>& error_state,
                                      Vector3d&    sensor_translation_drift,
                                      Quaterniond& sensor_rotation_drift)
{
  inject_error_state(error_state);

  sensor_translation_drift += error_state.block<3, 1>(DRIFT_TRANS_IDX, 0);

  const auto& angle_axis_drift_vector = error_state.block<3, 1>(DRIFT_ROT_IDX, 0);
  Quaterniond delta_q_drift;
  delta_q_drift = Eigen::AngleAxisd(angle_axis_drift_vector.norm(),
                                    angle_axis_drift_vector.normalized());

  sensor_rotation_drift = sensor_rotation_drift * delta_q_drift;
  m_est_quaternion      = m_est_quaternion * delta_q_drift;
  sensor_rotation_drift.normalize();
  m_est_quaternion.normalize();
}

void EsEkf2::poseMeasurementUpdate(const Matrix3d& R_cov, const Matrix<double, 3, 1>& y)
{
  // Definition on pg. 62 (272) onwards

  // measurement model jacobian (278) - (279)
  // X_{error_x} = I, H = H_{x}
  // TODO(lmark): Maybe add h_jac as a constant - always the same!
  Matrix<double, 3, N_STATES> h_jac = MatrixXd::Zero(3, N_STATES);
  h_jac.block<3, 3>(0, 0)           = Identity3x3;

  auto K = m_p_covariance * h_jac.transpose()
           * (h_jac * m_p_covariance * h_jac.transpose() + R_cov).inverse();

  // delta_x -> Error state
  Matrix<double, N_STATES, 1> delta_x = K * (y - m_est_position);

  // Better numerical stability, pg. 63. Footnote "Joseph form"
  m_p_covariance =
    (IdentityNxN - K * h_jac) * m_p_covariance * (IdentityNxN - K * h_jac).transpose()
    + K * R_cov * K.transpose();

  inject_error_state(delta_x);
}

void EsEkf2::angleMeasurementUpdate(const Matrix3d& R_cov, const Quaterniond& y)
{
  Matrix<double, 3, N_STATES> h_jac    = MatrixXd::Zero(3, N_STATES);
  h_jac.block<3, 3>(0, ANGLE_AXIS_IDX) = Identity3x3;

  AngleAxisd angle_axis_error(m_est_quaternion.conjugate() * y);
  auto       K = m_p_covariance * h_jac.transpose()
           * (h_jac * m_p_covariance * h_jac.transpose() + R_cov).inverse();
  auto delta_theta = angle_axis_error.angle() * angle_axis_error.axis();
  Matrix<double, N_STATES, 1> delta_x = K * delta_theta;

  m_p_covariance =
    (IdentityNxN - K * h_jac) * m_p_covariance * (IdentityNxN - K * h_jac).transpose()
    + K * R_cov * K.transpose();

  inject_error_state(delta_x);
}

void EsEkf2::angleMeasurementUpdateDrift(const Matrix3d&    R_cov,
                                         const Quaterniond& y,
                                         Vector3d&          est_position_drift,
                                         Quaterniond&       est_quaternion_drift,
                                         Matrix3d&          position_drift_cov,
                                         Matrix3d&          rotation_drift_cov)
{
  // Sola equation:(278)
  Matrix<double, 3, N_STATES + 2> H = MatrixXd::Zero(3, N_STATES + 2);
  // We measure quaternions directly this a standard
  // measurement model Jacobian for an extended Kalman filter

  // These blocks are calculated according to pg. 44
  H.block<3, 3>(0, ANGLE_AXIS_IDX) = Identity3x3;
  H.block<3, 3>(0, DRIFT_ROT_IDX)  = y.normalized().toRotationMatrix().transpose();

  Matrix<double, N_STATES + 2, N_STATES> H_dx =
    Matrix<double, N_STATES + 2, N_STATES>::Zero(N_STATES + 2, N_STATES);
  // Scola equation:(280)
  H_dx.block<6, 6>(POSITION_IDX, POSITION_IDX) = Matrix<double, 6, 6>::Identity(6, 6);
  H_dx.block<4, 3>(ANGLE_AXIS_IDX, ANGLE_AXIS_IDX) =
    0.5 * sf::firstOrderApproxLocal(m_est_quaternion);
  H_dx.block<12, 12>(ACC_BIAS_IDX + 1, ACC_BIAS_IDX) = MatrixXd::Identity(12, 12);
  H_dx.block<4, 3>(DRIFT_ROT_IDX + 1, DRIFT_ROT_IDX) =
    0.5 * sf::firstOrderApproxLocal(est_quaternion_drift);

  // TODO(lmark): This covariance "switching" based on sensor is probably not right.
  // What about prediction ? Which sensor drift covariance are we estimating there?
  // Solution: Reserve a variable amount of space for sensor drift.

  // Set drift covariance, and save previously stored covariance
  auto ph_pos = m_p_covariance.block<3, 3>(DRIFT_TRANS_IDX, DRIFT_TRANS_IDX);
  auto ph_rot = m_p_covariance.block<3, 3>(DRIFT_ROT_IDX, DRIFT_ROT_IDX);
  m_p_covariance.block<3, 3>(DRIFT_TRANS_IDX, DRIFT_TRANS_IDX) = position_drift_cov;
  m_p_covariance.block<3, 3>(DRIFT_ROT_IDX, DRIFT_ROT_IDX)     = rotation_drift_cov;

  auto h_jac = H * H_dx;
  auto K     = m_p_covariance * h_jac.transpose()
           * (h_jac * m_p_covariance * h_jac.transpose() + R_cov).inverse();

  AngleAxisd angle_axis_error((est_quaternion_drift * m_est_quaternion).conjugate() * y);
  auto       delta_theta = angle_axis_error.angle() * angle_axis_error.axis();
  Matrix<double, N_STATES, 1> delta_x = K * delta_theta;

  m_p_covariance =
    (IdentityNxN - K * h_jac) * m_p_covariance * (IdentityNxN - K * h_jac).transpose()
    + K * R_cov * K.transpose();

  // Set new calculated covariance, restore previously stored
  position_drift_cov = m_p_covariance.block<3, 3>(DRIFT_TRANS_IDX, DRIFT_TRANS_IDX);
  rotation_drift_cov = m_p_covariance.block<3, 3>(DRIFT_ROT_IDX, DRIFT_ROT_IDX);
  m_p_covariance.block<3, 3>(DRIFT_TRANS_IDX, DRIFT_TRANS_IDX) = ph_pos;
  m_p_covariance.block<3, 3>(DRIFT_ROT_IDX, DRIFT_ROT_IDX)     = ph_rot;
  inject_error_state_drift(delta_x, est_position_drift, est_quaternion_drift);
}

void EsEkf2::poseMeasurementUpdateDrift(const Matrix3d&             R_cov,
                                        const Matrix<double, 3, 1>& y,
                                        Vector3d&                   est_position_drift,
                                        Quaterniond&                est_quaternion_drift,
                                        Matrix3d&                   position_drift_cov,
                                        Matrix3d&                   rotation_drift_cov)
{
  // pg. 63 (278)
  //  + 2 - This H is wrt. the "true" UAV state quaternion-based representation
  Matrix<double, 3, N_STATES + 2> H     = MatrixXd::Zero(3, N_STATES + 2);
  H.block<3, 3>(0, POSITION_IDX)        = est_quaternion_drift.toRotationMatrix();// (170)
  H.block<3, 3>(0, DRIFT_TRANS_IDX + 1) = Identity3x3;// Position drift

  // Jacobian with respect to the quaternion pg 41 (174)
  H.block<3, 4>(0, DRIFT_ROT_IDX + 1) =
    sf::JacobianWithRespectToQuat(est_quaternion_drift, m_est_position);

  Matrix<double, N_STATES + 2, N_STATES> H_dx = MatrixXd::Zero(N_STATES + 2, N_STATES);
  // Scola equation:(280)
  H_dx.block<6, 6>(0, 0)    = MatrixXd::Identity(6, 6);
  H_dx.block<4, 3>(6, 6)    = 0.5 * sf::firstOrderApprox(m_est_quaternion);
  H_dx.block<12, 12>(10, 9) = MatrixXd::Identity(12, 12);
  H_dx.block<4, 3>(22, 21)  = 0.5 * sf::firstOrderApproxLocal(est_quaternion_drift);

  // Set drift covariance
  auto ph_pos = m_p_covariance.block<3, 3>(DRIFT_TRANS_IDX, DRIFT_TRANS_IDX);
  auto ph_rot = m_p_covariance.block<3, 3>(DRIFT_ROT_IDX, DRIFT_ROT_IDX);
  m_p_covariance.block<3, 3>(DRIFT_TRANS_IDX, DRIFT_TRANS_IDX) = position_drift_cov;
  m_p_covariance.block<3, 3>(DRIFT_ROT_IDX, DRIFT_ROT_IDX)     = rotation_drift_cov;

  auto h_jac = H * H_dx;
  auto K     = m_p_covariance * h_jac.transpose()
           * (h_jac * m_p_covariance * h_jac.transpose() + R_cov).inverse();
  // delta_x -> Error state
  // #1 and #2 should be in the same coordinate system - they are!
  // 1) Don't apply transform to #2 - both in GLOBAL
  // 2) Make sure y is also in drifted coordinate system
  // Y IS ALREADY IN A DFRIFTED COORDINATE SYSATEM (if we assume the sensor drifts)
  Matrix<double, N_STATES, 1> delta_x =
    K
    * (y// #1
       - (est_quaternion_drift.toRotationMatrix() * m_est_position// #2
          + est_position_drift));

  m_p_covariance =
    (IdentityNxN - K * h_jac) * m_p_covariance * (IdentityNxN - K * h_jac).transpose()
    + K * R_cov * K.transpose();

  position_drift_cov = m_p_covariance.block<3, 3>(DRIFT_TRANS_IDX, DRIFT_TRANS_IDX);
  rotation_drift_cov = m_p_covariance.block<3, 3>(DRIFT_ROT_IDX, DRIFT_ROT_IDX);
  m_p_covariance.block<3, 3>(DRIFT_TRANS_IDX, DRIFT_TRANS_IDX) = ph_pos;
  m_p_covariance.block<3, 3>(DRIFT_ROT_IDX, DRIFT_ROT_IDX)     = ph_rot;
  inject_error_state_drift(delta_x, est_position_drift, est_quaternion_drift);
}
