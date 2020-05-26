#include <iostream>
#include <Eigen/Geometry>
#include <cmath>
#include "utils.hpp"
using std::cos;
using std::sin;
using std::fabs;
using namespace Eigen;

class Ekf{
public:
  Ekf(VehicleParams params){

    params_ = params;
    x_hat = params_.initial_state;
    J_tp_ = 0.0002;
    std::cout << Phi << '\n';
    H = MatrixXd::Zero(3, 12);
    H.topLeftCorner(3,3) = MatrixXd::Identity(3, 3);
    M = MatrixXd::Identity(3, 3);

    L(3,3) = 1;
    L(4,4) = 1;
    L(5,5) = 1;
    L(9,9) = 1;
    L(10,10) = 1;
    L(11,11) = 1;

    Q = params_.Q*L*params_.T;
    Q(5,5) = params_.Qz*params_.T;

    std::cout << " T init: \n" << params_.T << '\n';
    P_minus = MatrixXd::Zero(12, 12);
    P_plus = MatrixXd::Zero(12, 12);
  }

  Pose prediction_step(Eigen::Matrix<double, 4, 1>  U){
    Pose pose;
    double U1, U2, U3, U4, omega;
    U1 = params_.b*(pow(U(0),2) + pow(U(1),2)+ pow(U(2),2) + pow(U(3),2)); //0.00000686428

    U2 = params_.l * params_.b*(-pow(U(1),2) + pow(U(3),2));

    U3 = params_.l * params_.b*(-pow(U(0),2) + pow(U(2),2));

    U4 = params_.d *(-pow(U(0),2) + pow(U(1),2)- pow(U(2),2) + pow(U(3),2));

    omega = -U(0) + U(1) - U(2) + U(3);

      //TO DO: ovo treba bolje/ljepse
      Phi << 1, 0, 0, params_.T,0,0,0,0,0,0,0,0,
             0, 1, 0, 0, params_.T, 0,0,0,0,0,0,0,
             0, 0, 1, 0, 0, params_.T, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 1, 0, 0, (params_.T*U[1]*(cos(x_hat[6])*sin(x_hat[8]) - cos(x_hat[8])*sin(x_hat[6])*sin(x_hat[7])))/params_.m, (params_.T*U[1]*cos(x_hat[6])*cos(x_hat[7])*cos(x_hat[8]))/params_.m, (params_.T*U[1]*(cos(x_hat[8])*sin(x_hat[6]) - cos(x_hat[6])*sin(x_hat[7])*sin(x_hat[8])))/params_.m, 0, 0, 0,
             0, 0, 0, 0, 1, 0, -(params_.T*U[1]*(cos(x_hat[6])*cos(x_hat[8]) + cos(x_hat[7])*sin(x_hat[6])*sin(x_hat[8])))/params_.m, -(params_.T*U[1]*cos(x_hat[6])*sin(x_hat[7])*sin(x_hat[8]))/params_.m, (params_.T*U[1]*(sin(x_hat[6])*sin(x_hat[8]) + cos(x_hat[6])*cos(x_hat[7])*cos(x_hat[8])))/params_.m, 0, 0, 0,
             0, 0, 0, 0, 0, 1, -(params_.T*U[1]*cos(x_hat[7])*sin(x_hat[6]))/params_.m, -(params_.T*U[1]*cos(x_hat[6])*sin(x_hat[7]))/params_.m, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, 1, 0, 0, params_.T, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 1, 0, 0, params_.T, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, params_.T,
             0, 0, 0, 0, 0, 0, 0, 0, 0, 1, (params_.Iyy - params_.Izz)/params_.Ixx*params_.T*x_hat[11], (params_.Iyy - params_.Izz)/params_.Ixx*params_.T*x_hat[10],
             0, 0, 0, 0, 0, 0, 0, 0, 0, params_.T*x_hat[11]*(params_.Izz - params_.Ixx)/params_.Iyy, 1, (params_.Izz - params_.Ixx)/params_.Iyy*params_.T*x_hat[9],
             0, 0, 0, 0, 0, 0, 0, 0, 0, (params_.Ixx - params_.Iyy)/params_.Izz*params_.T*x_hat[10], (params_.Ixx - params_.Iyy)/params_.Izz*params_.T*x_hat[9], 1;

      X_minus << x_hat(3),
               x_hat(4),
               x_hat(5),
               (sin(x_hat(8))*sin(x_hat(6)) + cos(x_hat(8))*sin(x_hat(7))*cos(x_hat(6)))*U1/params_.m,
               (-cos(x_hat(8))*sin(x_hat(6)) + sin(x_hat(8))*sin(x_hat(7))*cos(x_hat(6)))*U1/params_.m,
               -params_.g + cos(x_hat(7))*cos(x_hat(6))*U1/params_.m,
               x_hat(9),
               x_hat(10),
               x_hat(11),
               U2/params_.Ixx + (params_.Iyy - params_.Izz)/params_.Ixx*x_hat[11]*x_hat[10] - omega*x_hat[10]*J_tp_/params_.Ixx,
               U3/params_.Iyy + x_hat[11]*x_hat[9]*(params_.Izz - params_.Ixx)/params_.Iyy + omega*x_hat[9]*J_tp_/params_.Iyy,
               U4/params_.Izz + x_hat[10]*x_hat[9]*(params_.Ixx - params_.Iyy)/params_.Izz;

      x_hat = x_hat + params_.T*X_minus;
      
      P_minus = Phi*P_plus*Phi.transpose() + L*Q*L.transpose(); //get prediction fpr Pk
    if (x_hat(2) < 0) x_hat(2) = 0;

      pose.x = x_hat[0];
      pose.y = x_hat[1];
      pose.z = x_hat[2];

    return pose;
  }

  Pose measurment_update(Eigen::Matrix<double, 3, 1> y,Eigen::Matrix<double, 3, 3>  R){

    R = R/params_.T;
    K = P_minus*H.transpose()*(H*P_minus*H.transpose()+M*R*M.transpose()).inverse();
    x_hat = x_hat + K*(y-H*x_hat);
    P_plus = (MatrixXd::Identity(12, 12)-K*H)*P_minus;
    P_minus = P_plus;  // ovo se radi jer ocekujemo vise od jednog measuremnt update poziva prilikom rada EKF-a


    Pose pose;
    pose.x = x_hat[0];
    pose.y = x_hat[1];
    pose.z = x_hat[2];

    return pose;
  }
  ~Ekf(){
  }

private:

    Eigen::Matrix<double, 12, 12> Phi; //matrix of estimate
    Eigen::Matrix<double, 12, 1>  x_hat; // estimate
    Eigen::Matrix<double, 3, 12>  H;
    Eigen::Matrix<double, 3, 3>  M;
    Eigen::Matrix<double, 12, 12>  L;
    Eigen::Matrix<double, 12, 12>  P_minus;
    Eigen::Matrix<double, 12, 12>  P_plus;
    Eigen::Matrix<double, 12, 1>  X_minus;
    Eigen::Matrix<double, 12, 12>  Q;
    Eigen::Matrix<double, 12, 3>  K;

    double J_tp_;

    Pose_vec model_pose;
    VehicleParams params_;

};
