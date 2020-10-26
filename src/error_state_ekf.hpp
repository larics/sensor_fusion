#ifndef SENSOR_FUSION_ES_EKF_HPP
#define SENSOR_FUSION_ES_EKF_HPP

#include <Eigen/Geometry>
#include "utils.hpp"
using namespace Eigen;
class EsEkf{
public:
		EsEkf(){
			//constants that dont change
			g << 0,0,-9.81; // gravity
			//motion model noise jacobian
			l_jac =  MatrixXd::Zero(9, 6);
			l_jac.bottomLeftCorner(6,6) =
							MatrixXd::Identity(6,6);

			// measurement model jacobian
			h_jac =  MatrixXd::Zero(3,9);
			h_jac.bottomLeftCorner(3,3) =
							MatrixXd::Identity(3,3);
			p_est << 0,0,0; v_est << 0,0,0; q_est << 0,0,0,0;
			p_cov = 10 * MatrixXd::Identity(9,9);
		}

		void setInitialState(Matrix<double, 10, 1> initial_state){
			p_est << initial_state(0),
							initial_state(1),
							initial_state(2);

			v_est << initial_state(3),
							initial_state(4),
							initial_state(5);

			q_est << initial_state(6),
							initial_state(7),
							initial_state(8),
							initial_state(9);
		}

		Matrix<double, 10, 1> predicition(
							Matrix<double, 3, 1> imu_f, Matrix<double, 3, 3> var_imu_f,
						  Matrix<double, 3, 1> imu_w, Matrix<double, 3, 3> var_imu_w,
						  double delta_t){

		Quaternion<double> q(q_est(0),q_est(1),
											 q_est(2),q_est(3));
    // 1. Update state with IMU inputs
		Matrix<double, 3, 3> R;
		R = q.toRotationMatrix();
		p_est = p_est + delta_t*v_est +
						pow(delta_t,2)/2 * (R * imu_f + g);
		v_est = v_est + delta_t * (R * imu_f + g);

		Quaternion<double> imu_q;
		imu_q = AngleAxisd(imu_w(0), Vector3d::UnitX())
				* AngleAxisd(imu_w(1), Vector3d::UnitY())
				* AngleAxisd(imu_w(2), Vector3d::UnitZ());

		Quaternion<double> q_est_temp = imu_q*q;
		q_est << q_est_temp.w(),
						 q_est_temp.x(),
						 q_est_temp.y(),
						 q_est_temp.z();

		//  Linearize the motion model and compute Jacobians
		Matrix<double, 9, 9> f_jac = MatrixXd::Identity(9,9);
		f_jac.block<3,3>(0,3) = delta_t* MatrixXd::Identity(3,3);
		f_jac.block<3,3>(3,6) =
		        -skew_symetric(R * imu_f) * delta_t;
		// 2. Propagate uncertainty
		Matrix<double, 6, 6> q_cov;
		q_cov.block<3,3>(0,0) = var_imu_f * pow(delta_t,2);
		q_cov.block<3,3>(3,3) = var_imu_w * pow(delta_t,2);

		p_cov = f_jac * p_cov * f_jac.transpose() +
						l_jac * q_cov * l_jac.transpose();

		p_est_pred = p_est; v_est_pred = v_est;
		q_est_pred = q_est;

		Matrix<double, 10, 1> state;
		state << p_est,v_est,q_est;
		return state;
		}

		Matrix<double, 10, 1> measurement_update(Matrix<double, 3, 3> sensor_var,
														Matrix<double, 3, 1> y){

			Matrix<double, 9, 3> K;
			K = p_cov * h_jac.transpose() *
					(h_jac*p_cov*h_jac.transpose() + sensor_var).inverse();

//			std::cout << "p_cov\n" << p_est << std::endl;
//			std::cout << "p_est_pred \n" << p_est << std::endl;
//			std::cout << "y 0 \n" << y << std::endl;
//			std::cout << "sensor_var \n" << sensor_var << std::endl;
//			std::cout << "K \n" << K << std::endl;

			Matrix<double, 9, 1> delta_x;
			//3.2 Compute error state
			//TODO mozda p_est_pre???
			delta_x = K * (y - p_est);

			// 3.3 Correct predicted state
			p_est = p_est + delta_x.block<3,1>(0,0);
			v_est = v_est + delta_x.block<3,1>(3,0);
			Matrix<double,4,1> quat_from_aa = axixs_angle2quat(
							delta_x.block<3,1>(6,0));

			Quaternion<double> q(quat_from_aa(0),
													 quat_from_aa(1),
													 quat_from_aa(2),
													 quat_from_aa(3));
			Quaternion<double> q_temp,q_est_obj(q_est(0),q_est(1),
																					q_est(2),q_est(3));
			q_temp = q_est_obj*q;
			q_est << q_temp.w(),
							 q_temp.x(),
							 q_temp.y(),
							 q_temp.z();

			Matrix<double, 10, 1> state;
			state << p_est,v_est,q_est;
			return state;
		}

		Matrix<double,10,1 > getState(){
			Matrix<double,10,1> state;
			state << p_est,v_est,q_est;
			return state;
		}

private:
		// error state Ekf
		Matrix<double, 3, 1>  g;
		Matrix<double, 9, 6>	l_jac;
		Matrix<double, 3, 9>	h_jac;
		Matrix<double, 3, 1>	p_est_pred, v_est_pred, p_est,v_est;
		Matrix<double, 4, 1>	q_est_pred,q_est;
		Matrix<double, 9, 9>  p_cov;
};

#endif //SENSOR_FUSION_ES_EKF_HPP