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
    std::cout << "Ekf" << '\n';
    x_hat << 0,0,0,0,0,0,0,0,0,0,0,0;
    //U << 0, 0, 0, 0;
    params_ = params;
    Ixx = 0.075032;
    Iyy = 0.075032;
    Izz = 0.15006;

    std::cout << Phi << '\n';
    H = MatrixXd::Zero(3, 12);
    H.topLeftCorner(3,3) = MatrixXd::Identity(3, 3);
    M = MatrixXd::Identity(3, 3);
    Theta = MatrixXd::Zero(12, 12);

    L(3,3) = 1;
    L(4,4) = 1;
    L(5,5) = 1;
    L(9,9) = 1;
    L(10,10) = 1;
    L(11,11) = 1;

    Q = 0.5*L*params_.T;
    Q(5,5) = 100*Q(5,5);

    // R1 = 0.0025*MatrixXd::Identity(3, 3)/params_.T;
    // R2 = 0.01*MatrixXd::Identity(3, 3)/params_.T;
    // R = R1*R1*R2*R2 * ((R1*R1+R2*R2).inverse());
    //R(2,2) = 0.001/params_.T;
    std::cout << " T init: \n" << params_.T << '\n';
    P_minus = MatrixXd::Zero(12, 12);
    P_plus = MatrixXd::Zero(12, 12);
  }

  Pose prediction_step(Eigen::Matrix<double, 4, 1>  U){
    Pose pose;

    //y = (y.transpose()*R1.inverse()+y2.transpose()*R2.inverse()) * ((R1.inverse()+R2.inverse()).inverse());
    U[1] = 0.00001819*(pow(U(0),2)+pow(U(1),2)+ //0.00000686428
                       pow(U(2),2)+pow(U(3),2));
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
             0, 0, 0, 0, 0, 0, 0, 0, 0, 1, (Iyy - Izz)/Ixx*params_.T*x_hat[11], (Iyy - Izz)/Ixx*params_.T*x_hat[10],
             0, 0, 0, 0, 0, 0, 0, 0, 0, params_.T*x_hat[11]*(Izz - Ixx)/Iyy, 1, (Izz - Ixx)/Iyy*params_.T*x_hat[9],
             0, 0, 0, 0, 0, 0, 0, 0, 0, (Ixx - Iyy)/Izz*params_.T*x_hat[10], (Ixx - Iyy)/Izz*params_.T*x_hat[9], 1;

      x_hat = Phi*x_hat; //get prediction for x_hat
      P_minus = Phi*P_plus*Phi.transpose() + L*Q*L.transpose(); //get prediction fpr Pk


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
    // Eigen::Matrix<double, 4, 1>  U; //motor_speeds
    Eigen::Matrix<double, 3, 12>  H;
    Eigen::Matrix<double, 3, 3>  M;
    Eigen::Matrix<double, 12, 12>  L;
    Eigen::Matrix<double, 12, 12>  Theta;
    Eigen::Matrix<double, 12, 12>  P_minus;
    Eigen::Matrix<double, 12, 12>  P_plus;
    Eigen::Matrix<double, 12, 1>  X_mius;
    Eigen::Matrix<double, 12, 12>  Q;
    Eigen::Matrix<double, 12, 3>  K;

    Eigen::Matrix<double, 3, 3>  R1,R2;

    double Ixx,Iyy,Izz;

    Pose_vec model_pose;
    VehicleParams params_;

};
