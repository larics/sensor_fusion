//
// Created by kovac on 05. 12. 2020..
//

#include "filter.h"

EsEkf2::EsEkf2(EsEkfParams params) {
  g_est = params.g;
  // motion model noise jacobian
  l_jac = MatrixXd::Zero(N_STATES, 12);
  l_jac.block<12, 12>(3, 0) = MatrixXd::Identity(12, 12);

  // initial state for pose, linear velocity and orientation
  p_est = {0, 0, 0};
  v_est = {0, 0, 0};
  q_est = {1, 0, 0, 0};

  // initial state for imu accelerometer and gyroscope bias
  fb_est = {0, 0, 0};
  var_imu_wb = params.wb_var;
  wb_est = {0, 0, 0};
  var_imu_fb = params.fb_var;
  // g_est = { 0,0,0};

  // drift
  p_drift = {0, 0, 0};
  Matrix3d m = MatrixXd::Identity(3, 3);
  q_drift = m;

  // initital state error is zero (can be anything else),18 nuber of core states
  // p,v,q,ab,wb,g
  p_cov = MatrixXd::Zero(N_STATES, N_STATES);
  p_cov.block<3, 3>(0, 0) = params.init_pose_p_cov * Matrix3d::Identity();
  p_cov.block<3, 3>(3, 3) = params.init_v_p_cov * Matrix3d::Identity();
  p_cov.block<3, 3>(6, 6) = params.init_q_p_cov * Matrix3d::Identity();
  p_cov.block<3, 3>(9, 9) = params.init_ab_p_cov * Matrix3d::Identity();
  p_cov.block<3, 3>(12, 12) = params.init_wb_p_cov * Matrix3d::Identity();
  p_cov.block<3, 3>(15, 15) = params.init_g_cov * Matrix3d::Identity();

  p_cov.block<3, 3>(18, 18) = params.init_p_drift * MatrixXd::Identity(3, 3);
  p_cov.block<3, 3>(21, 21) = params.init_q_drift * MatrixXd::Identity(3, 3);

  if (!params.estimate_acc_bias) {
    p_cov.block<3, 3>(9, 9) = MatrixXd::Zero(3, 3);
    var_imu_fb = Matrix3d::Zero();
  }
  if (!params.estimate_gyro_bias) {
    p_cov.block<3, 3>(12, 12) = MatrixXd::Zero(3, 3);
    var_imu_wb = Matrix3d::Zero();
  }
  if (!params.estimate_gravity) {
    p_cov.block<3, 3>(15, 15) = MatrixXd::Zero(3, 3);
  }
  std::cout << "l_jacobian:\n"
            << l_jac << '\n'
            << "p_cov init:\n"
            << p_cov << '\n';
}

void EsEkf2::prediction(Matrix<double, 3, 1> imu_f, Matrix3d var_imu_f,
                        Matrix<double, 3, 1> imu_w, Matrix3d var_imu_w,
                        double delta_t) {
  // 1. Update state with IMU inputs
  p_est.vector() = p_est.vector() + delta_t * v_est.vector() +
                   pow(delta_t, 2) / 2 *
                       (q_est.toRotationMatrix() * (imu_f - fb_est.vector()) +
                        (g_est.vector()));

  v_est.vector() = v_est.vector() + delta_t * (q_est.toRotationMatrix() *
                                                   (imu_f - fb_est.vector()) +
                                               (g_est.vector()));

  Quaterniond imu_q = euler2quat({delta_t * (imu_w[0] - wb_est.x()),
                                  delta_t * (imu_w[1] - wb_est.y()),
                                  delta_t * (imu_w[2] - wb_est.z())});

  q_est = q_est * imu_q;
  q_est.normalize();

  // 1.1 Linearize the motion model
  // and compute Jacobians (Scola (311))
  Matrix<double, N_STATES, N_STATES> f_jac;
  f_jac = MatrixXd::Identity(N_STATES, N_STATES);
  f_jac.block<3, 3>(0, 3) = delta_t * MatrixXd::Identity(3, 3);
  // TODO transform imu maybe beore prediction????
  f_jac.block<3, 3>(3, 6) =
      -skew_symetric(q_est.toRotationMatrix() * (imu_f - fb_est.vector())) *
      delta_t;
  f_jac.block<3, 3>(3, 9) = -q_est.toRotationMatrix() * delta_t;
  f_jac.block<3, 3>(3, 15) = delta_t * MatrixXd::Identity(3, 3);
  f_jac.block<3, 3>(6, 12) = -q_est.toRotationMatrix() * delta_t;
  f_jac.block<3, 3>(6, 6) = MatrixXd::Identity(3, 3);

  // std::cout << "f_jac:\n" << f_jac.block<12,12>(0,0) << "\n";

  // 2. Propagate uncertainty
  Matrix<double, 12, 12> q_cov = MatrixXd::Identity(12, 12);
  q_cov.block<3, 3>(0, 0) = var_imu_f * pow(delta_t, 2);
  q_cov.block<3, 3>(3, 3) = var_imu_w * pow(delta_t, 2);
  q_cov.block<3, 3>(6, 6) = var_imu_fb * delta_t;
  q_cov.block<3, 3>(9, 9) = var_imu_wb * delta_t;

  Matrix<double, 24, 24> p_cov_temp = p_cov;
  p_cov = f_jac * p_cov * f_jac.transpose() + l_jac * q_cov * l_jac.transpose();

  ROS_INFO_THROTTLE(2.0, "Prediction");
}

void EsEkf2::poseMeasurementUpdate(Matrix3d R_cov, Matrix<double, 3, 1> y) {
  // measurement model jacobian
  Matrix<double, 3, N_STATES> h_jac = MatrixXd::Zero(3, N_STATES);
  h_jac.block<3, 3>(0, 0) = MatrixXd::Identity(3, 3);

  Matrix<double, N_STATES, 3> K = MatrixXd::Zero(N_STATES, 3);
  K = p_cov * h_jac.transpose() *
      (h_jac * p_cov * h_jac.transpose() + R_cov).inverse();

  // delta_x -> Error state
  Matrix<double, N_STATES, 1> delta_x;

  // 3.2 Compute error state
  delta_x = K * (y - p_est.vector());

  // 3.3 Correct predicted state
  p_est.vector() = p_est.vector() + delta_x.block<3, 1>(0, 0);
  v_est.vector() = v_est.vector() + delta_x.block<3, 1>(3, 0);
  Matrix<double, 4, 1> quat_from_aa =
      axixs_angle2quat(delta_x.block<3, 1>(6, 0));
  Quaterniond q(quat_from_aa(0), quat_from_aa(1), quat_from_aa(2),
                quat_from_aa(3));

  q_est = q * q_est;
  q_est.normalize();

  fb_est.vector() = fb_est.vector() + delta_x.block<3, 1>(9, 0);
  wb_est.vector() = wb_est.vector() + delta_x.block<3, 1>(12, 0);
  g_est.vector() = g_est.vector() + delta_x.block<3, 1>(15, 0);

  //	std::cout << "fb_est-> " << fb_est.vector().transpose() << '\n'
  //						<< "wb_est-> " << wb_est.vector().transpose()
  //<<
  //'\n'
  //					  << "g_est-> " << g_est.vector().transpose()
  //<<
  //'\n'; ROS_INFO("Pose measurement");
  p_cov = (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac) * p_cov *
              (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac).transpose() +
          K * R_cov * K.transpose();
}

void EsEkf2::angleMeasurementUpdate(Matrix<double, 4, 4> R_cov, Quaterniond y) {
  // Sola equation:(278)
  Matrix<double, 4, N_STATES + 2> H = MatrixXd::Zero(4, N_STATES + 2);
  // We measure quaternions directly this a standard
  // measurement model Jacobian for an extended Kalman filter
  H.block<4, 4>(0, 6) = rightQuatProdMat(q_drift);
  H.block<4, 4>(0, 22) = leftQuatProdMat(q_est);

  Matrix<double, N_STATES + 2, N_STATES> H_dx =
      MatrixXd::Zero(N_STATES + 2, N_STATES);
  // Scola equation:(280)
  H_dx.block<6, 6>(0, 0) = MatrixXd::Identity(6, 6);
  H_dx.block<4, 3>(6, 6) = 0.5 * firstOrderApprox(q_est);
  H_dx.block<12, 12>(10, 9) = MatrixXd::Identity(12, 12);
  H_dx.block<4, 3>(22, 21) = 0.5 * firstOrderApproxLocal(q_drift);

  Matrix<double, 4, N_STATES> h_jac = MatrixXd::Zero(4, N_STATES);

  h_jac = H * H_dx;
  Matrix<double, N_STATES, 4> K = MatrixXd::Zero(N_STATES, 4);
  K = p_cov * h_jac.transpose() *
      (h_jac * p_cov * h_jac.transpose() + R_cov).inverse();

  q_est = q_est * q_drift;

  Matrix<double, N_STATES, 1> delta_x;
  Matrix<double, 4, 1> delta_quat;
  delta_quat << y.w() - q_est.w(), y.x() - q_est.x(), y.y() - q_est.y(),
      y.z() - q_est.z();
  delta_x = K * (delta_quat);

  p_est.vector() = p_est.vector() + delta_x.block<3, 1>(0, 0);
  v_est.vector() = v_est.vector() + delta_x.block<3, 1>(3, 0);

  Matrix<double, 4, 1> quat_from_aa =
      axixs_angle2quat(delta_x.block<3, 1>(6, 0));
  Quaterniond q(quat_from_aa(0), quat_from_aa(1), quat_from_aa(2),
                quat_from_aa(3));

  q_est = q * q_est;
  q_est.normalize();

  fb_est.vector() = fb_est.vector() + delta_x.block<3, 1>(9, 0);
  wb_est.vector() = wb_est.vector() + delta_x.block<3, 1>(12, 0);
  g_est.vector() = g_est.vector() + delta_x.block<3, 1>(15, 0);
  p_drift.vector() = p_drift.vector() + delta_x.block<3, 1>(18, 0);
  Matrix<double, 4, 1> quat_from_aa2 =
      axixs_angle2quat(delta_x.block<3, 1>(21, 0));
  Quaterniond q2(quat_from_aa2(0), quat_from_aa2(1), quat_from_aa2(2),
                 quat_from_aa2(3));

  q_drift = q_drift * q2;
  q_drift.normalize();

  p_cov = (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac) * p_cov *
              (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac).transpose() +
          K * R_cov * K.transpose();
}

void EsEkf2::poseMeasurementUpdateDrift(Matrix3d R_cov,
                                        Matrix<double, 3, 1> y) {
  Matrix<double, 3, N_STATES + 2> H = MatrixXd::Zero(3, N_STATES);
  H.block<3, 3>(0, 0) = q_drift.toRotationMatrix();

  H.block<3, 3>(0, 19) = MatrixXd::Identity(3, 3);
  H.block<3, 4>(0, 22) = JacobianWithRespectToQuat(q_drift, p_est.vector());

  Matrix<double, N_STATES + 2, N_STATES> H_dx =
      MatrixXd::Zero(N_STATES + 2, N_STATES);
  // Scola equation:(280)
  H_dx.block<6, 6>(0, 0) = MatrixXd::Identity(6, 6);
  H_dx.block<4, 3>(6, 6) = 0.5 * firstOrderApprox(q_est);
  H_dx.block<12, 12>(10, 9) = MatrixXd::Identity(12, 12);
  H_dx.block<4, 3>(22, 21) = 0.5 * firstOrderApproxLocal(q_drift);

  Matrix<double, 3, N_STATES> h_jac = MatrixXd::Zero(3, N_STATES);

  h_jac = H * H_dx;

  Matrix<double, N_STATES, 3> K = MatrixXd::Zero(N_STATES, 3);
  K = p_cov * h_jac.transpose() *
      (h_jac * p_cov * h_jac.transpose() + R_cov).inverse();
  // delta_x -> Error state
  Matrix<double, N_STATES, 1> delta_x;

  delta_x =
      K *
      (y - (q_drift.toRotationMatrix() * p_est.vector() + p_drift.vector()));

  // 3.3 Correct predicted state
  p_est.vector() = p_est.vector() + delta_x.block<3, 1>(0, 0);
  v_est.vector() = v_est.vector() + delta_x.block<3, 1>(3, 0);
  Matrix<double, 4, 1> quat_from_aa =
      axixs_angle2quat(delta_x.block<3, 1>(6, 0));
  Quaterniond q(quat_from_aa(0), quat_from_aa(1), quat_from_aa(2),
                quat_from_aa(3));

  q_est = q * q_est;
  q_est.normalize();

  fb_est.vector() = fb_est.vector() + delta_x.block<3, 1>(9, 0);
  wb_est.vector() = wb_est.vector() + delta_x.block<3, 1>(12, 0);
  g_est.vector() = g_est.vector() + delta_x.block<3, 1>(15, 0);

  p_drift.vector() = p_drift.vector() + delta_x.block<3, 1>(18, 0);

  quat_from_aa = axixs_angle2quat(delta_x.block<3, 1>(21, 0));
  Quaternion<double> q2(quat_from_aa(0), quat_from_aa(1), quat_from_aa(2),
                        quat_from_aa(3));

  q_drift = q_drift * q2;
  q_drift.normalize();

  p_cov = (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac) * p_cov *
              (MatrixXd::Identity(N_STATES, N_STATES) - K * h_jac).transpose() +
          K * R_cov * K.transpose();
}
