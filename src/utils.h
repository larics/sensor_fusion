#ifndef SENSOR_FUSION_UTILS_H
#define SENSOR_FUSION_UTILS_H

#include <Eigen/Geometry>

namespace sf {
using namespace Eigen;
// Utilities for the sensor fusion algorithm

//! Wrap angles to [-pi, pi)
/*!
    Some discussion about alternative implementations:
    https://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi
 */
double               wrapToPi(double angle);
Matrix<double, 3, 3> skew_symetric(const Matrix<double, 3, 1>& m);
Matrix<double, 4, 1> axixs_angle2quat(const Matrix<double, 3, 1>& axis_angle);
Quaternion<double>   euler2quat(const Matrix<double, 3, 1>& euler);
Matrix<double, 4, 3> firstOrderApprox(const Quaterniond& q);
Matrix<double, 4, 3> firstOrderApproxLocal(const Quaterniond& q);
Matrix<double, 4, 4> leftQuatProdMat(const Quaterniond& q);
Matrix<double, 4, 4> rightQuatProdMat(const Quaterniond& q);
Matrix<double, 3, 4> JacobianWithRespectToQuat(const Quaterniond& q, const Vector3d& a);
}// namespace sf

#endif// SENSOR_FUSION_UTILS_H
