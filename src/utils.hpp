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
};

struct Pose{
  double x,y,z;
};
struct velocitiy{
  double x,y,z;
};

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
