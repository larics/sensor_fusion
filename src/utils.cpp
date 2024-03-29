#include "utils.h"
#include <Eigen/Geometry>

using namespace Eigen;
// Utilities for the sensor fusion algorithm

//! Wrap angles to [-pi, pi)
/*!
    Some discussion about alternative implementations:
    https://stackoverflow.com/questions/4633177/c-how-to-wrap-a-float-to-the-interval-pi-pi
 */
double sf::wrapToPi(double angle)
{
  // Naive solution assumes that we're wrapping regularly,
  // So that the angle argument will never be much bigger than 2pi
  // or much smaller than -2pi

  while (angle >= M_PI) angle -= 2 * M_PI;
  while (angle < -M_PI) angle += 2 * M_PI;

  return angle;
}

Matrix<double, 3, 3> sf::skew_symetric(const Matrix<double, 3, 1>& m)
{
  Matrix<double, 3, 3> a;
  a << 0, -m(2), m(1), m(2), 0, -m(0), -m(1), m(0), 0;
  return a;
}

Matrix<double, 4, 1> sf::axixs_angle2quat(const Matrix<double, 3, 1>& axis_angle)
{
  double norm = axis_angle.norm();
  double w, x, y, z;
  w = cos(norm / 2);
  if (norm < 1e-10) {
    x = 0;
    y = 0;
    z = 0;
  } else {
    x = axis_angle[0] / norm * sin(norm / 2);
    y = axis_angle[1] / norm * sin(norm / 2);
    z = axis_angle[2] / norm * sin(norm / 2);
  }
  Matrix<double, 4, 1> result;
  result << w, x, y, z;
  return result;
}

Quaternion<double> sf::euler2quat(const Matrix<double, 3, 1>& euler)
{
  double roll  = euler(0);
  double pitch = euler(1);
  double yaw   = euler(2);

  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);

  double w = cr * cp * cy + sr * sp * sy;
  double x = sr * cp * cy - cr * sp * sy;
  double y = cr * sp * cy + sr * cp * sy;
  double z = cr * cp * sy - sr * sp * cy;

  return Quaternion<double>(w, x, y, z);
}

Matrix<double, 4, 3> sf::firstOrderApprox(const Quaterniond& q)
{
  Matrix<double, 4, 3> m;
  m << -q.x(), -q.y(), -q.z(), q.w(), q.z(), -q.y(), -q.z(), q.w(), q.x(), q.y(), -q.x(),
    q.w();
  return m;
}

Matrix<double, 4, 3> sf::firstOrderApproxLocal(const Quaterniond& q)
{
  Matrix<double, 4, 3> m;
  m << -q.x(), -q.y(), -q.z(), q.w(), -q.z(), q.y(), q.z(), q.w(), -q.x(), -q.y(), q.x(),
    q.w();
  return m;
}

Matrix<double, 4, 4> sf::leftQuatProdMat(const Quaterniond& q)
{
  Matrix<double, 4, 4> m;
  m << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), -q.z(), q.y(), q.y(), q.z(), q.w(),
    -q.x(), q.z(), -q.y(), q.x(), q.w();
  return m;
}

Matrix<double, 4, 4> sf::rightQuatProdMat(const Quaterniond& q)
{
  Matrix<double, 4, 4> m;
  m << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), q.z(), -q.y(), q.y(), -q.z(), q.w(),
    q.x(), q.z(), q.y(), -q.x(), q.w();
  return m;
}

Matrix<double, 3, 4> sf::JacobianWithRespectToQuat(const Quaterniond& q,
                                                   const Vector3d&    a)
{
  Vector3d v  = { q.x(), q.y(), q.z() };
  double   w  = q.w();
  Vector3d dw = 2 * (w * a + v.cross(a));
  // std::cout << "dw-> " << dw.transpose() << '\n';
  Matrix3d I  = Matrix3d::Identity();
  Matrix3d dv = 2
                * (v.transpose() * a * I + v * a.transpose() - a * v.transpose()
                   - w * sf::skew_symetric(a));

  // std::cout << "Dv->\n" << dv << '\n';

  Matrix<double, 3, 4> m = MatrixXd::Zero(3, 4);
  m.block<3, 1>(0, 0)    = dw;
  // std::cout << "M\n" << m << '\n';
  m.bottomRightCorner(3, 3) = dv;
  // std::cout << "M\n" << m << '\n';
  return m;
}
