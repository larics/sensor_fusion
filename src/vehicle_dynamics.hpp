#include <iostream>
#include <Eigen/Geometry>
#include <cmath>
#include "utils.hpp"
using std::cos;
using std::sin;
using std::fabs;

using namespace Eigen;

class VehicleDynamics{
public:
  VehicleDynamics(){
    pose_.x = 0;
    pose_.y = 0;
    pose_.z = 0;
    v_.x = 0;
    v_.y = 0;
    v_.z = 0;
    T = 0.04;
    m = 2.3;
  }
  ~VehicleDynamics(){
    std::cout << "destructor" << std::endl;
  }
  Pose integrate_step(double& phi,double& theta, double& psi,
                      Eigen::Matrix<double, 1, 4> motor_speed)
  {
    U1 = 0.000009119*(pow(motor_speed(0),2)+pow(motor_speed(1),2)+ //0.00000686428
                       pow(motor_speed(2),2)+pow(motor_speed(3),2));

    std::cout << "U1 " << U1 << '\n';

    X_ddot = (sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi))*U1/m;
    Y_ddot = (-cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi))*U1/m;
    Z_ddot = -9.81+(cos(theta)*cos(phi))*U1/m;

    std::cout << "sila up " << (cos(theta)*cos(phi))*U1/m << '\n';
    if (fabs(X_ddot) >= 0.05) {
      v_.x = v_.x + X_ddot*T;
      pose_.x = pose_.x + v_.x*T;
    }
    if (fabs(Y_ddot) >= 0.05) {
      v_.y = v_.y + Y_ddot*T;
      pose_.y = pose_.y + v_.y*T;
    }

    if (fabs(Z_ddot) >= 0.05) {
      v_.z = v_.z + Z_ddot*T;
      pose_.z = pose_.z + v_.z*T;
    }
    if(pose_.z < 0 && v_.z < 0){
     pose_.z = 0;
     v_.z = 0;
    }


    return pose_;
  }

  double X_ddot,Y_ddot,Z_ddot;

  Pose pose_;
  velocitiy v_;
  double T;
  double U1,m;
};
