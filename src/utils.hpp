#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>
#include <iterator>
#include <numeric>
#include <fstream>
//https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

struct Quaternions {
    double w, x, y, z;
};

struct EulerAngles {
    double roll, pitch, yaw;
};

struct VehicleParams{
  double m; //mass
  double g; //gravity

  double Ixx; //Inertia
  double Iyy;
  double Izz;
  double T;
  // correlation matrix of the model
  double Q;
  double Qz; //for the z axis that is a bit worse than the rest
};

struct Pose{
  double x,y,z;
};
struct velocitiy{
  double x,y,z;
};

struct Pose_vec{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;
};

struct SensorParams{
  std::string id;
  std::string topic;
  // 1 if its an odom sensor, gives delta values not absolute values
  bool is_odom;
  Eigen::Matrix<double, 3, 3>  R; // correlation matrix of the sensor
};

std::ostream& save_vector_as_matrix( const std::string& name,
                                     Pose_vec& pose_vec)
{
	// stm << name_tag << name << '\n' << type_tag << '\n' << rows_tag << '\n'
	//     << "# columns: " << matrix.size() << '\n' ;
  std::ofstream file(name+"_x.mat");
  std::copy( pose_vec.x.begin(), pose_vec.x.end(), std::ostream_iterator<double>( file, " " ) ) ;
	file << "\n\n\n" ;
  std::ofstream filey(name+"_y.mat");
  std::copy( pose_vec.y.begin(), pose_vec.y.end(), std::ostream_iterator<double>( filey, " " ) ) ;
	filey << "\n\n\n" ;
  std::ofstream filez(name+"_z.mat");
  std::copy( pose_vec.z.begin(), pose_vec.z.end(), std::ostream_iterator<double>( filez, " " ) ) ;
	filez << "\n\n\n" ;
}

// Save vector as matrix type in Octave (.mat) stm.
// http://www.cplusplus.com/forum/general/221958/

namespace
{
    const std::string name_tag = "# name: " ;
    const std::string type_tag = "# type: matrix" ;
    const std::string rows_tag = "# rows: 1" ;
}

template <typename T>
std::ostream& save_vector_as_matrix( const std::string& name, const std::vector<T>& matrix, std::ofstream& stm )
{
	// stm << name_tag << name << '\n' << type_tag << '\n' << rows_tag << '\n'
	//     << "# columns: " << matrix.size() << '\n' ;

    std::copy( matrix.begin(), matrix.end(), std::ostream_iterator<T>( stm, " " ) ) ;
	return stm << "\n\n\n" ;
}


EulerAngles ToEulerAngles(Quaternions q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}
