//
// Created by kovac on 05. 12. 2020..
//

#include "filter.h"

EsEkf2::EsEkf2(const EsEkfParams& params)
{
  m_est_gravity = params.g;
  // motion model noise jacobian
  m_jacobian                     = MatrixXd::Zero(N_STATES, 12);
  m_jacobian.block<12, 12>(3, 0) = MatrixXd::Identity(12, 12);

  // initial state for pose, linear velocity and orientation
  m_est_position     = { 0, 0, 0 };
  m_est_lin_velocity = { 0, 0, 0 };
  m_est_quaternion   = { 1, 0, 0, 0 };

  // initial state for imu accelerometer and gyroscope bias
  m_est_acc_bias       = { 0, 0, 0 };
  m_acc_bias_variance  = params.acc_bias_variance;
  m_gyro_bias_variance = params.gyro_bias_variance;
  m_est_gyro_bias      = { 0, 0, 0 };
  // m_est_gravity = { 0,0,0};

  // drift
  m_position_drift       = { 0, 0, 0 };
  m_est_quaternion_drift = I3x3;

  // initital state error is zero (can be anything else),18 nuber of core states
  // p,v,q,ab,wb,g
  m_p_covariance                     = MatrixXd::Zero(N_STATES, N_STATES);
  m_p_covariance.block<3, 3>(0, 0)   = params.init_pose_p_cov * I3x3;
  m_p_covariance.block<3, 3>(3, 3)   = params.init_v_p_cov * I3x3;
  m_p_covariance.block<3, 3>(6, 6)   = params.init_q_p_cov * I3x3;
  m_p_covariance.block<3, 3>(9, 9)   = params.init_ab_p_cov * I3x3;
  m_p_covariance.block<3, 3>(12, 12) = params.init_wb_p_cov * I3x3;
  m_p_covariance.block<3, 3>(15, 15) = params.init_g_cov * I3x3;

  m_p_covariance.block<3, 3>(18, 18) = params.init_p_drift * I3x3;
  m_p_covariance.block<3, 3>(21, 21) = params.init_q_drift * I3x3;

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
  // 1. Update state with IMU inputs
  auto rot_est = m_est_quaternion.toRotationMatrix();

  m_est_position.vector() =
    m_est_position.vector() + delta_t * m_est_lin_velocity.vector()
    + pow(delta_t, 2) / 2.0
        * (rot_est * (imu_f - m_est_acc_bias.vector()) + (m_est_gravity.vector()));

  m_est_lin_velocity.vector() =
    m_est_lin_velocity.vector()
    + delta_t * (rot_est * (imu_f - m_est_acc_bias.vector()) + (m_est_gravity.vector()));

  Quaterniond imu_q = euler2quat({ delta_t * (imu_w[0] - m_est_gyro_bias.x()),
                                   delta_t * (imu_w[1] - m_est_gyro_bias.y()),
                                   delta_t * (imu_w[2] - m_est_gyro_bias.z()) });

  m_est_quaternion = m_est_quaternion * imu_q;
  m_est_quaternion.normalize();
  rot_est = m_est_quaternion.toRotationMatrix();

  // 1.1 Linearize the motion model
  // and compute Jacobians (Scola (311))
  Matrix<double, N_STATES, N_STATES> f_jac =
    Matrix<double, N_STATES, N_STATES>::Identity();
  f_jac.block<3, 3>(0, 3) = delta_t * I3x3;
  // TODO transform imu maybe beore prediction????
  f_jac.block<3, 3>(3, 6) =
    -skew_symetric(rot_est * (imu_f - m_est_acc_bias.vector())) * delta_t;
  f_jac.block<3, 3>(3, 9)  = -rot_est * delta_t;
  f_jac.block<3, 3>(3, 15) = delta_t * I3x3;
  f_jac.block<3, 3>(6, 12) = f_jac.block<3, 3>(3, 9);
  f_jac.block<3, 3>(6, 6)  = I3x3;

  // std::cout << "f_jac:\n" << f_jac.block<12,12>(0,0) << "\n";

  // 2. Propagate uncertainty
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
  // measurement model jacobian
  Matrix<double, 3, N_STATES> h_jac = MatrixXd::Zero(3, N_STATES);
  h_jac.block<3, 3>(0, 0)           = I3x3;

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
  Matrix<double, 4, 1> quat_from_aa = axixs_angle2quat(delta_x.block<3, 1>(6, 0));
  Quaterniond q(quat_from_aa(0), quat_from_aa(1), quat_from_aa(2), quat_from_aa(3));

  m_est_quaternion = q * m_est_quaternion;
  m_est_quaternion.normalize();

  m_est_acc_bias.vector()  = m_est_acc_bias.vector() + delta_x.block<3, 1>(9, 0);
  m_est_gyro_bias.vector() = m_est_gyro_bias.vector() + delta_x.block<3, 1>(12, 0);
  m_est_gravity.vector()   = m_est_gravity.vector() + delta_x.block<3, 1>(15, 0);

  //	std::cout << "m_est_acc_bias-> " << m_est_acc_bias.vector().transpose() << '\n'
  //						<< "m_est_gyro_bias-> " << m_est_gyro_bias.vector().transpose()
  //<<
  //'\n'
  //					  << "m_est_gravity-> " << m_est_gravity.vector().transpose()
  //<<
  //'\n'; ROS_INFO("Pose measurement");
  m_p_covariance = (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac) * m_p_covariance
                     * (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac).transpose()
                   + K * R_cov * K.transpose();
}

void EsEkf2::angleMeasurementUpdate(const Matrix<double, 4, 4>& R_cov,
                                    const Quaterniond&          y)
{
  // Sola equation:(278)
  Matrix<double, 4, N_STATES + 2> H = MatrixXd::Zero(4, N_STATES + 2);
  // We measure quaternions directly this a standard
  // measurement model Jacobian for an extended Kalman filter
  H.block<4, 4>(0, 6)  = rightQuatProdMat(m_est_quaternion_drift);
  H.block<4, 4>(0, 22) = leftQuatProdMat(m_est_quaternion);

  Matrix<double, N_STATES + 2, N_STATES> H_dx = MatrixXd::Zero(N_STATES + 2, N_STATES);
  // Scola equation:(280)
  H_dx.block<6, 6>(0, 0)    = MatrixXd::Identity(6, 6);
  H_dx.block<4, 3>(6, 6)    = 0.5 * firstOrderApprox(m_est_quaternion);
  H_dx.block<12, 12>(10, 9) = MatrixXd::Identity(12, 12);
  H_dx.block<4, 3>(22, 21)  = 0.5 * firstOrderApproxLocal(m_est_quaternion_drift);

  Matrix<double, 4, N_STATES> h_jac = MatrixXd::Zero(4, N_STATES);

  h_jac                         = H * H_dx;
  Matrix<double, N_STATES, 4> K = MatrixXd::Zero(N_STATES, 4);
  K                             = m_p_covariance * h_jac.transpose()
      * (h_jac * m_p_covariance * h_jac.transpose() + R_cov).inverse();

  m_est_quaternion = m_est_quaternion * m_est_quaternion_drift;

  Matrix<double, N_STATES, 1> delta_x;
  Matrix<double, 4, 1>        delta_quat;
  delta_quat << y.w() - m_est_quaternion.w(), y.x() - m_est_quaternion.x(),
    y.y() - m_est_quaternion.y(), y.z() - m_est_quaternion.z();
  delta_x = K * (delta_quat);

  m_est_position.vector()     = m_est_position.vector() + delta_x.block<3, 1>(0, 0);
  m_est_lin_velocity.vector() = m_est_lin_velocity.vector() + delta_x.block<3, 1>(3, 0);

  Matrix<double, 4, 1> quat_from_aa = axixs_angle2quat(delta_x.block<3, 1>(6, 0));
  Quaterniond q(quat_from_aa(0), quat_from_aa(1), quat_from_aa(2), quat_from_aa(3));

  m_est_quaternion = q * m_est_quaternion;
  m_est_quaternion.normalize();

  m_est_acc_bias.vector()   = m_est_acc_bias.vector() + delta_x.block<3, 1>(9, 0);
  m_est_gyro_bias.vector()  = m_est_gyro_bias.vector() + delta_x.block<3, 1>(12, 0);
  m_est_gravity.vector()    = m_est_gravity.vector() + delta_x.block<3, 1>(15, 0);
  m_position_drift.vector() = m_position_drift.vector() + delta_x.block<3, 1>(18, 0);
  Matrix<double, 4, 1> quat_from_aa2 = axixs_angle2quat(delta_x.block<3, 1>(21, 0));
  Quaterniond q2(quat_from_aa2(0), quat_from_aa2(1), quat_from_aa2(2), quat_from_aa2(3));

  m_est_quaternion_drift = m_est_quaternion_drift * q2;
  m_est_quaternion_drift.normalize();

  m_p_covariance = (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac) * m_p_covariance
                     * (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac).transpose()
                   + K * R_cov * K.transpose();
}

void EsEkf2::poseMeasurementUpdateDrift(const Matrix3d&             R_cov,
                                        const Matrix<double, 3, 1>& y)
{
  Matrix<double, 3, N_STATES + 2> H = MatrixXd::Zero(3, N_STATES + 2);
  H.block<3, 3>(0, 0)               = m_est_quaternion_drift.toRotationMatrix();

  H.block<3, 3>(0, 19) = I3x3;
  H.block<3, 4>(0, 22) =
    JacobianWithRespectToQuat(m_est_quaternion_drift, m_est_position.vector());

  Matrix<double, N_STATES + 2, N_STATES> H_dx = MatrixXd::Zero(N_STATES + 2, N_STATES);
  // Scola equation:(280)
  H_dx.block<6, 6>(0, 0)    = MatrixXd::Identity(6, 6);
  H_dx.block<4, 3>(6, 6)    = 0.5 * firstOrderApprox(m_est_quaternion);
  H_dx.block<12, 12>(10, 9) = MatrixXd::Identity(12, 12);
  H_dx.block<4, 3>(22, 21)  = 0.5 * firstOrderApproxLocal(m_est_quaternion_drift);

  Matrix<double, 3, N_STATES> h_jac = MatrixXd::Zero(3, N_STATES);

  h_jac = H * H_dx;

  Matrix<double, N_STATES, 3> K = MatrixXd::Zero(N_STATES, 3);
  K                             = m_p_covariance * h_jac.transpose()
      * (h_jac * m_p_covariance * h_jac.transpose() + R_cov).inverse();
  // delta_x -> Error state
  Matrix<double, N_STATES, 1> delta_x;

  delta_x = K
            * (y
               - (m_est_quaternion_drift.toRotationMatrix() * m_est_position.vector()
                  + m_position_drift.vector()));

  // 3.3 Correct predicted state
  m_est_position.vector()     = m_est_position.vector() + delta_x.block<3, 1>(0, 0);
  m_est_lin_velocity.vector() = m_est_lin_velocity.vector() + delta_x.block<3, 1>(3, 0);
  Matrix<double, 4, 1> quat_from_aa = axixs_angle2quat(delta_x.block<3, 1>(6, 0));
  Quaterniond q(quat_from_aa(0), quat_from_aa(1), quat_from_aa(2), quat_from_aa(3));

  m_est_quaternion = q * m_est_quaternion;
  m_est_quaternion.normalize();

  m_est_acc_bias.vector()  = m_est_acc_bias.vector() + delta_x.block<3, 1>(9, 0);
  m_est_gyro_bias.vector() = m_est_gyro_bias.vector() + delta_x.block<3, 1>(12, 0);
  m_est_gravity.vector()   = m_est_gravity.vector() + delta_x.block<3, 1>(15, 0);

  m_position_drift.vector() = m_position_drift.vector() + delta_x.block<3, 1>(18, 0);

  quat_from_aa = axixs_angle2quat(delta_x.block<3, 1>(21, 0));
  Quaternion<double> q2(
    quat_from_aa(0), quat_from_aa(1), quat_from_aa(2), quat_from_aa(3));

  m_est_quaternion_drift = m_est_quaternion_drift * q2;
  m_est_quaternion_drift.normalize();

  m_p_covariance = (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac) * m_p_covariance
                     * (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac).transpose()
                   + K * R_cov * K.transpose();
}
