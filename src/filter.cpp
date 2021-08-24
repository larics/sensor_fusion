//
// Created by kovac on 05. 12. 2020..
//

#include "filter.h"

EsEkf2::EsEkf2(const EsEkfParams& params)
{

  // motion model noise jacobian
  m_jacobian                     = MatrixXd::Zero(N_STATES, 12);
  m_jacobian.block<12, 12>(3, 0) = MatrixXd::Identity(12, 12);

  // initial state for pose, linear velocity and orientation
  m_est_position       = { 0, 0, 0 };
  m_est_lin_velocity   = { 0, 0, 0 };
  m_est_quaternion     = { 1, 0, 0, 0 };
  m_est_acc_bias       = { 0, 0, 0 };
  m_acc_bias_variance  = params.acc_bias_variance;
  m_gyro_bias_variance = params.gyro_bias_variance;
  m_est_gyro_bias      = { 0, 0, 0 };
  m_est_gravity        = params.g;

  // initital state error is zero (can be anything else),18 nuber of core states
  // p,v,q,ab,wb,g
  m_p_covariance                     = IdentityNxN;
  m_p_covariance.block<3, 3>(0, 0)   = params.init_pose_p_cov * Identity3x3;
  m_p_covariance.block<3, 3>(3, 3)   = params.init_v_p_cov * Identity3x3;
  m_p_covariance.block<3, 3>(6, 6)   = params.init_q_p_cov * Identity3x3;
  m_p_covariance.block<3, 3>(9, 9)   = params.init_ab_p_cov * Identity3x3;
  m_p_covariance.block<3, 3>(12, 12) = params.init_wb_p_cov * Identity3x3;
  m_p_covariance.block<3, 3>(15, 15) = params.init_g_cov * Identity3x3;
  m_p_covariance.block<3, 3>(18, 18) = params.init_p_drift * Identity3x3;
  m_p_covariance.block<3, 3>(21, 21) = params.init_q_drift * Identity3x3;

  if (!params.estimate_acc_bias) {
    m_p_covariance.block<3, 3>(9, 9) = MatrixXd::Zero(3, 3);
    m_acc_bias_variance              = Matrix3d::Zero();
  }
  if (!params.estimate_gyro_bias) {
    m_p_covariance.block<3, 3>(12, 12) = MatrixXd::Zero(3, 3);
    m_gyro_bias_variance               = Matrix3d::Zero();
  }
  if (!params.estimate_gravity) {
    m_p_covariance.block<3, 3>(15, 15) = MatrixXd::Zero(3, 3);
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
  // 1. Update nominal state with IMU inputs
  auto rot_est = m_est_quaternion.toRotationMatrix();

  m_est_position.vector() +=
    +delta_t * m_est_lin_velocity.vector()
    + pow(delta_t, 2) / 2.0
        * (rot_est * (imu_f - m_est_acc_bias.vector()) + m_est_gravity.vector());

  m_est_lin_velocity.vector() +=
    +delta_t * (rot_est * (imu_f - m_est_acc_bias.vector()) + (m_est_gravity.vector()));

  Quaterniond delta_q;
  delta_q = AngleAxisd(delta_t * (imu_w[0] - m_est_gyro_bias.x()), Vector3d::UnitX())
            * AngleAxisd(delta_t * (imu_w[1] - m_est_gyro_bias.y()), Vector3d::UnitY())
            * AngleAxisd(delta_t * (imu_w[2] - m_est_gyro_bias.z()), Vector3d::UnitZ());
  m_est_quaternion = m_est_quaternion * delta_q;
  m_est_quaternion.normalize();
  rot_est = m_est_quaternion.toRotationMatrix();

  // 1.1 Linearize the motion model
  // and compute Jacobians (Scola (311))
  Matrix<double, N_STATES, N_STATES> f_jac = IdentityNxN;
  f_jac.block<3, 3>(0, 3)                  = delta_t * Identity3x3;
  // TODO transform imu maybe beore prediction????
  f_jac.block<3, 3>(3, 6) =
    -sf::skew_symetric(rot_est * (imu_f - m_est_acc_bias.vector())) * delta_t;
  f_jac.block<3, 3>(3, 9)  = -rot_est * delta_t;
  f_jac.block<3, 3>(3, 15) = delta_t * Identity3x3;
  f_jac.block<3, 3>(6, 12) = f_jac.block<3, 3>(3, 9);

  // TODO(lmark): Maybe comment out this line, should set at #83
  f_jac.block<3, 3>(6, 6) = Identity3x3;

  // std::cout << "f_jac:\n" << f_jac.block<12,12>(0,0) << "\n";

  // 2. Propagate uncertainty
  // Components on pg. 60 (262-265)
  // Definition on pg. 70 (312)
  Matrix<double, 12, 12> q_cov = Matrix<double, 12, 12>::Identity();
  q_cov.block<3, 3>(0, 0)      = var_imu_f * pow(delta_t, 2);
  q_cov.block<3, 3>(3, 3)      = var_imu_w * pow(delta_t, 2);
  q_cov.block<3, 3>(6, 6)      = m_acc_bias_variance * delta_t;
  q_cov.block<3, 3>(9, 9)      = m_gyro_bias_variance * delta_t;
  m_p_covariance               = f_jac * m_p_covariance * f_jac.transpose()
                   + m_jacobian * q_cov * m_jacobian.transpose();

  ROS_INFO_THROTTLE(2.0, "EsEkf2::prediction()");
}

void EsEkf2::poseMeasurementUpdate(const Matrix3d& R_cov, const Matrix<double, 3, 1>& y)
{
  // Definition on pg. 62 (272) onwards

  // measurement model jacobian (278) - (279)
  // X_{error_x} = I, H = H_{x}
  // TODO(lmark): Maybe add h_jac as a constant - always the same!
  Matrix<double, 3, N_STATES> h_jac = MatrixXd::Zero(3, N_STATES);
  h_jac.block<3, 3>(0, 0)           = Identity3x3;

  Matrix<double, N_STATES, 3> K = MatrixXd::Zero(N_STATES, 3);
  K                             = m_p_covariance * h_jac.transpose()
      * (h_jac * m_p_covariance * h_jac.transpose() + R_cov).inverse();

  // delta_x -> Error state
  Matrix<double, N_STATES, 1> delta_x;

  // 3.2 Compute error state
  delta_x = K * (y - m_est_position.vector());

  // 3.3 Correct predicted state
  m_est_position.vector()     = m_est_position.vector() + delta_x.block<3, 1>(0, 0);
  m_est_lin_velocity.vector() = m_est_lin_velocity.vector() + delta_x.block<3, 1>(3, 0);

  const auto& angle_axis_vector = delta_x.block<3, 1>(6, 0);
  Quaterniond delta_q;
  delta_q = Eigen::AngleAxisd(angle_axis_vector.norm(), angle_axis_vector.normalized());

  // pg. 68 (303)
  m_est_quaternion = delta_q * m_est_quaternion;
  m_est_quaternion.normalize();

  m_est_acc_bias.vector()  = m_est_acc_bias.vector() + delta_x.block<3, 1>(9, 0);
  m_est_gyro_bias.vector() = m_est_gyro_bias.vector() + delta_x.block<3, 1>(12, 0);
  m_est_gravity.vector()   = m_est_gravity.vector() + delta_x.block<3, 1>(15, 0);

  // Better numerical stability, pg. 63. Footnote "Joseph form"
  m_p_covariance =
    (IdentityNxN - K * h_jac) * m_p_covariance * (IdentityNxN - K * h_jac).transpose()
    + K * R_cov * K.transpose();
}

void EsEkf2::angleMeasurementUpdate(const Matrix<double, 4, 4>& R_cov,
                                    const Quaterniond&          y)
{
  // Sola equation:(278)
  Matrix<double, 4, N_STATES + 2> H = Matrix<double, 4, N_STATES + 2>::Zero();
  H.block<4, 4>(0, 6)               = Matrix4d::Identity();

  Matrix<double, N_STATES + 2, N_STATES> H_dx =
    Matrix<double, N_STATES + 2, N_STATES>::Zero(N_STATES + 2, N_STATES);
  // Scola equation:(280)
  H_dx.block<6, 6>(0, 0)    = MatrixXd::Identity(6, 6);
  H_dx.block<4, 3>(6, 6)    = 0.5 * sf::firstOrderApprox(m_est_quaternion);
  H_dx.block<12, 12>(10, 9) = MatrixXd::Identity(12, 12);

  auto h_jac = H * H_dx;
  auto K     = m_p_covariance * h_jac.transpose()
           * (h_jac * m_p_covariance * h_jac.transpose() + R_cov).inverse();

  // TODO(lmark): This is wrong but it somehow works ?!
  Matrix<double, N_STATES, 1> delta_x;
  Matrix<double, 4, 1>        delta_quat;
  delta_quat << y.w() - m_est_quaternion.w(), y.x() - m_est_quaternion.x(),
    y.y() - m_est_quaternion.y(), y.z() - m_est_quaternion.z();
  delta_x = K * (delta_quat);

  m_est_position.vector()     = m_est_position.vector() + delta_x.block<3, 1>(0, 0);
  m_est_lin_velocity.vector() = m_est_lin_velocity.vector() + delta_x.block<3, 1>(3, 0);

  const auto& angle_axis_vector = delta_x.block<3, 1>(6, 0);
  Quaterniond delta_q;
  delta_q = Eigen::AngleAxisd(angle_axis_vector.norm(), angle_axis_vector.normalized());
  m_est_quaternion = delta_q * m_est_quaternion;
  m_est_quaternion.normalize();

  m_est_acc_bias.vector()  = m_est_acc_bias.vector() + delta_x.block<3, 1>(9, 0);
  m_est_gyro_bias.vector() = m_est_gyro_bias.vector() + delta_x.block<3, 1>(12, 0);
  m_est_gravity.vector()   = m_est_gravity.vector() + delta_x.block<3, 1>(15, 0);

  m_p_covariance =
    (IdentityNxN - K * h_jac) * m_p_covariance * (IdentityNxN - K * h_jac).transpose()
    + K * R_cov * K.transpose();
}

void EsEkf2::angleMeasurementUpdateDrift(const Matrix<double, 4, 4>& R_cov,
                                         const Quaterniond&          y,
                                         Translation3d&              est_position_drift,
                                         Quaterniond&                est_quaternion_drift)
{
  // Sola equation:(278)
  Matrix<double, 4, N_STATES + 2> H = MatrixXd::Zero(4, N_STATES + 2);
  // We measure quaternions directly this a standard
  // measurement model Jacobian for an extended Kalman filter

  // TODO(lmark) check if m_est_quaternion_drift <-> m_est_quaternion
  H.block<4, 4>(0, 6)  = sf::rightQuatProdMat(est_quaternion_drift);
  H.block<4, 4>(0, 22) = sf::leftQuatProdMat(m_est_quaternion);

  Matrix<double, N_STATES + 2, N_STATES> H_dx =
    Matrix<double, N_STATES + 2, N_STATES>::Zero(N_STATES + 2, N_STATES);
  // Scola equation:(280)
  H_dx.block<6, 6>(0, 0)    = Matrix<double, 6, 6>::Identity(6, 6);
  H_dx.block<4, 3>(6, 6)    = 0.5 * sf::firstOrderApprox(m_est_quaternion);
  H_dx.block<12, 12>(10, 9) = MatrixXd::Identity(12, 12);
  H_dx.block<4, 3>(22, 21)  = 0.5 * sf::firstOrderApproxLocal(est_quaternion_drift);

  auto h_jac = H * H_dx;
  auto K     = m_p_covariance * h_jac.transpose()
           * (h_jac * m_p_covariance * h_jac.transpose() + R_cov).inverse();

  // TODO(lmark): This is wrong but it somehow works ?!
  Matrix<double, N_STATES, 1> delta_x;
  Matrix<double, 4, 1>        delta_quat;
  delta_quat << y.w() - m_est_quaternion.w(), y.x() - m_est_quaternion.x(),
    y.y() - m_est_quaternion.y(), y.z() - m_est_quaternion.z();
  delta_x = K * (delta_quat);

  m_est_position.vector()     = m_est_position.vector() + delta_x.block<3, 1>(0, 0);
  m_est_lin_velocity.vector() = m_est_lin_velocity.vector() + delta_x.block<3, 1>(3, 0);

  const auto& angle_axis_vector = delta_x.block<3, 1>(6, 0);
  Quaterniond delta_q;
  delta_q = Eigen::AngleAxisd(angle_axis_vector.norm(), angle_axis_vector.normalized());
  m_est_quaternion = delta_q * m_est_quaternion;
  m_est_quaternion.normalize();

  m_est_acc_bias.vector()     = m_est_acc_bias.vector() + delta_x.block<3, 1>(9, 0);
  m_est_gyro_bias.vector()    = m_est_gyro_bias.vector() + delta_x.block<3, 1>(12, 0);
  m_est_gravity.vector()      = m_est_gravity.vector() + delta_x.block<3, 1>(15, 0);
  est_position_drift.vector() = est_position_drift.vector() + delta_x.block<3, 1>(18, 0);

  const auto& angle_axis_drift_vector = delta_x.block<3, 1>(21, 0);
  Quaterniond delta_q_drift;
  delta_q_drift = Eigen::AngleAxisd(angle_axis_drift_vector.norm(),
                                    angle_axis_drift_vector.normalized());

  est_quaternion_drift = est_quaternion_drift * delta_q_drift;
  m_est_quaternion     = m_est_quaternion * delta_q_drift;
  est_quaternion_drift.normalize();
  m_est_quaternion.normalize();

  m_p_covariance =
    (IdentityNxN - K * h_jac) * m_p_covariance * (IdentityNxN - K * h_jac).transpose()
    + K * R_cov * K.transpose();
}

void EsEkf2::poseMeasurementUpdateDrift(const Matrix3d&             R_cov,
                                        const Matrix<double, 3, 1>& y,
                                        Translation3d&              est_position_drift,
                                        Quaterniond&                est_quaternion_drift)
{
  // pg. 63 (278)
  //  + 2 - This H is wrt. the "true" UAV state quaternion-based representation
  Matrix<double, 3, N_STATES + 2> H = MatrixXd::Zero(3, N_STATES + 2);
  H.block<3, 3>(0, 0)               = est_quaternion_drift.toRotationMatrix();// (170)
  H.block<3, 3>(0, 19)              = Identity3x3;// Position drift

  // Jacobian with respect to the quaternion pg 41 (174)
  H.block<3, 4>(0, 22) =
    sf::JacobianWithRespectToQuat(est_quaternion_drift, m_est_position.vector());

  Matrix<double, N_STATES + 2, N_STATES> H_dx = MatrixXd::Zero(N_STATES + 2, N_STATES);
  // Scola equation:(280)
  H_dx.block<6, 6>(0, 0)    = MatrixXd::Identity(6, 6);
  H_dx.block<4, 3>(6, 6)    = 0.5 * sf::firstOrderApprox(m_est_quaternion);
  H_dx.block<12, 12>(10, 9) = MatrixXd::Identity(12, 12);
  H_dx.block<4, 3>(22, 21)  = 0.5 * sf::firstOrderApproxLocal(est_quaternion_drift);

  Matrix<double, 3, N_STATES> h_jac = MatrixXd::Zero(3, N_STATES);
  h_jac                             = H * H_dx;

  Matrix<double, N_STATES, 3> K = MatrixXd::Zero(N_STATES, 3);
  K                             = m_p_covariance * h_jac.transpose()
      * (h_jac * m_p_covariance * h_jac.transpose() + R_cov).inverse();
  // delta_x -> Error state
  Matrix<double, N_STATES, 1> delta_x;

  // #1 and #2 should be in the same coordinate system - they are!
  // 1) Don't apply transform to #2 - both in GLOBAL
  // 2) Make sure y is also in drifted coordinate system
  // Y IS ALREADY IN A DFRIFTED COORDINATE SYSATEM (if we assume the sensor drifts)
  delta_x = K
            * (y// #1
               - (est_quaternion_drift.toRotationMatrix() * m_est_position.vector()// #2
                  + est_position_drift.vector()));

  // 3.3 Correct predicted state
  m_est_position.vector()     = m_est_position.vector() + delta_x.block<3, 1>(0, 0);
  m_est_lin_velocity.vector() = m_est_lin_velocity.vector() + delta_x.block<3, 1>(3, 0);

  const auto& angle_axis_vector = delta_x.block<3, 1>(6, 0);
  Quaterniond delta_q;
  delta_q = Eigen::AngleAxisd(angle_axis_vector.norm(), angle_axis_vector.normalized());

  m_est_quaternion = delta_q * m_est_quaternion;
  m_est_quaternion.normalize();

  m_est_acc_bias.vector()     = m_est_acc_bias.vector() + delta_x.block<3, 1>(9, 0);
  m_est_gyro_bias.vector()    = m_est_gyro_bias.vector() + delta_x.block<3, 1>(12, 0);
  m_est_gravity.vector()      = m_est_gravity.vector() + delta_x.block<3, 1>(15, 0);
  est_position_drift.vector() = est_position_drift.vector() + delta_x.block<3, 1>(18, 0);

  const auto& angle_axis_drift_vector = delta_x.block<3, 1>(21, 0);
  Quaterniond delta_q_drift;
  delta_q_drift = Eigen::AngleAxisd(angle_axis_drift_vector.norm(),
                                    angle_axis_drift_vector.normalized());

  est_quaternion_drift = est_quaternion_drift * delta_q_drift;
  est_quaternion_drift.normalize();

  m_p_covariance =
    (IdentityNxN - K * h_jac) * m_p_covariance * (IdentityNxN - K * h_jac).transpose()
    + K * R_cov * K.transpose();
}
