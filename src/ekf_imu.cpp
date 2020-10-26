////
//// Created by kovac on 10. 09. 2020..
////
//
//#include "ekf_imu.h"
//
////Constructor using params
//EkfImu::EkfImu(VehicleParams params) {
//	state = params.initial_state_imu;
//
//	phi = MatrixXd::Identity(6, 6);
//	phi.topRightCorner(3,3) = MatrixXd::Identity(3, 3)*params.T;
//
//	gamma << 0.5*pow(params.T,2), 0, 0,
//					 0, 0.5 * pow(params.T,2), 0,
//					 0, 0, 0.5 * pow(params.T,2),
//					 params.T, 0, 0,
//					 0, params.T, 0,
//					 0, 0, params.T;
//	std::cout << "PHI \n" << phi << "\n";
//	std::cout << "Gamma \n" << gamma << "\n";
//
//	Q = MatrixXd::Identity(6,6);
//	Q(0,0) = params.Qx; Q(1,1) = params.Qy; Q(2,2) = params.Qz;
//	Q(3,3) = params.Qx_angle; Q(4,4) = params.Qy_angle; Q(5,5) = params.Qz_angle;
//	H = MatrixXd::Zero(3,6);
//	H.topLeftCorner(3,3) = MatrixXd::Identity(3, 3);
//	P = MatrixXd::Zero(6, 6);
//
//	std::cout << "Q \n" << Q << "\n";
//	std::cout << "H \n" << H << "\n";
//
//}
//
//void EkfImu::predictionStep(Eigen::Matrix<double, 3, 1> acc) {
//
//	std::cout << "acc\n" << acc << "\n";
//	state = phi*state + gamma*acc;
//	P_minus = phi*P*phi.transpose() + Q;
//}
//
//Pose EkfImu::measurmentUpdate(Eigen::Matrix<double, 3, 1> sensor,
//															Eigen::Matrix<double, 3, 3> R) {
//
//	K = P_minus*H.transpose()*(H*P_minus*H.transpose()+R).inverse();
//
//	state = state + K*(sensor-H*state);
//
//	P = (MatrixXd::Identity(6, 6)-K*H)*P_minus*
//					(MatrixXd::Identity(6, 6)-K*H).transpose()
//					+ K*R*K.transpose();
//
//	P_minus = P;
//
//	Pose pose;
//	pose.x = state[0];
//	pose.y = state[1];
//	pose.z = state[2];
//	return pose;
//
//}