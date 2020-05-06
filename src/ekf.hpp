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

    Q = 0.05*L*params_.T;
    Q(5,5) = 100*Q(5,5);
    R = 0.0025*MatrixXd::Identity(3, 3)/params_.T;
    //R(2,2) = 0.001/params_.T;
    std::cout << " T init: \n" << params_.T << '\n';
    std::cout << " R init: \n" << R << '\n';
    P_minus = MatrixXd::Zero(12, 12);
    P_plus = MatrixXd::Zero(12, 12);
  }

  Pose prediction_step(Eigen::Matrix<double, 3, 1> y, Eigen::Matrix<double, 4, 1>  U){
    Pose pose;
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
    //   std::cout << "--------izraz----------"
    //             << '\n' << Ixx
    //             << '\n' << Iyy
    //             << '\n' << Izz;
    // std::cout << "--------izraz----------"
    //           << '\n' << params_.T*x_hat[11] <<'\n';
    // std::cout << "--------izraz----------"
    //           << '\n' << params_.T*x_hat[11]*(Izz - Ixx)/Iyy <<'\n';

      // std::cout << " Phi prediction: \n" << Phi << '\n';
      x_hat = Phi*x_hat; //get prediction for x_hat
      P_minus = Phi*P_plus*Phi.transpose() + L*Q*L.transpose(); //get prediction fpr Pk
      // std::cout << " x_hat prediction: \n" << x_hat << '\n';


      K = P_minus*H.transpose()*(H*P_minus*H.transpose()+M*R*M.transpose()).inverse();
      // std::cout << " K prediction: \n" << K << '\n';
      // std::cout << " P_minus prediction: \n" << P_minus << '\n';
      // std::cout << " H prediction: \n" << H << '\n';
      // std::cout << " M prediction: \n" << M << '\n';
      // std::cout << " R prediction: \n" << R << '\n';
      x_hat = x_hat + K*(y-H*x_hat); // measurement update
      P_plus = (MatrixXd::Identity(12, 12)-K*H)*P_minus; //measurement update

      model_pose.x.push_back(K(0,0));
      model_pose.y.push_back(K(1,1));
      model_pose.z.push_back(K(2,2));

      pose.x = x_hat[0];
      pose.y = x_hat[1];
      pose.z = x_hat[2];

    // std::cout << "Motor sped " << U << '\n';
    // std::cout << "Pose with sensors \nX: " << y[0] << '\n'
    //           << "Y: " << y[1] << '\n'
    //           << "Z: " << y[2] << '\n';
    return pose;
  }

  void measurment_step(){

  }
  ~Ekf(){
    std::string name = "model";
    save_vector_as_matrix(name,model_pose);
  }

private:
  // void init_params(VehicleParams params){
  //   params_.m = params.m;
  //   params_.g = params.g;
  //   params_.Ixx = params.Ixx;
  //   params_.Iyy = params.Iyy;
  //   params_.Izz = params.Izz;
  //   params_.T = params.T;
  // }

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
    Eigen::Matrix<double, 3, 3>  R;
    Eigen::Matrix<double, 12, 3>  K;

    double Ixx,Iyy,Izz;

    Pose_vec model_pose;
    VehicleParams params_;

};
