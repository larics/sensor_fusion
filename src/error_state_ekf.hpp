#ifndef SENSOR_FUSION_ES_EKF_HPP
#define SENSOR_FUSION_ES_EKF_HPP

#include <Eigen/Geometry>
#include "utils.hpp"
using namespace Eigen;
class EsEkf{
public:
		EsEkf(){
			g<< 0,0,-9.81;
			l_jac=  MatrixXd::Zero(9, 6);  // motion model noise jacobian
			l_jac.bottomLeftCorner(6,6) =
							MatrixXd::Identity(6,6);

			h_jac;  //measurement model jacobian
			h_jac =  MatrixXd::Zero(3,9);
			h_jac.bottomLeftCorner(3,3) = MatrixXd::Identity(3,3);

			p_est << -4.38368, -1.14485,20.044;
			v_est << -0.97,1.49,0.04 ;
			q_est << 1,0,0,0;
			p_cov = MatrixXd::Identity(9,9);
			initialized = false;
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
		void setPest(Matrix<double, 3, 1> initial_state){
			p_est << initial_state(0),
							initial_state(1),
							initial_state(2);
		}
		void setvest(Matrix<double, 3, 1> initial_state){
			v_est << initial_state(0),
							initial_state(1),
							initial_state(2);
		}
		void setQest(Matrix<double, 4, 1> initial_state){
			q_est << initial_state(0),
							initial_state(1),
							initial_state(2),
							initial_state(3);
			std::cout << "INITAL STATE  " << q_est.transpose() << std::endl;
		}
		bool isIinit(){return initialized;}

		Matrix<double, 10, 1> predicition(
						Matrix<double, 3, 1> imu_f,
						double var_imu_f,
						Matrix<double, 3, 1> imu_w,
						double var_imu_w,
						double delta_t){
			initialized = true;
			Quaternion<double> q(q_est(0),q_est(1),
													 q_est(2),q_est(3));
			// 1. Update state with IMU inputs
			Matrix3d transform_imu = q.toRotationMatrix();
			p_est = p_est + delta_t*v_est +
							pow(delta_t,2)/2 * (transform_imu * imu_f + g);

//			std::cout << "delta_t " << delta_t << "\n"
//								<< "v_est " << v_est.transpose() << "\n"
//								<< "delta_t*v_est " << delta_t*v_est << "\n"
//								<< " transform imu \n" << transform_imu << "\n"
//								<< " imu_f " << imu_f.transpose() << "\n"
//								<< " g " << g.transpose() << "\n"
//								<< "acc\n" << pow(delta_t,2)/2 * (transform_imu * imu_f + g) << "\n"
//								<< "P_EST -> " << p_est;



			v_est = v_est + delta_t * (transform_imu * imu_f + g);

			Quaternion<double> imu_q = euler2quat({delta_t*imu_w(0),
																						 delta_t*imu_w(1),
																						 delta_t*imu_w(2)});

			//std::cout << "imu_w -> " << imu_w.transpose() << std::endl;
			//std::cout << "imu_q -> " << imu_q.w() << imu_q.x() << imu_q.y() << imu_q.z() << std::endl;
			imu_q = q*imu_q;
			imu_q = imu_q.normalized();
			//std::cout << "imu_q2 -> " << imu_q.w() << imu_q.x() << imu_q.y() << imu_q.z() << "\n" << std::endl;
			q_est << imu_q.w(),imu_q.x(),imu_q.y(),imu_q.z();

			//1.1 Linearize the motion model and compute Jacobians
			f_jac = MatrixXd::Identity(9,9);
			f_jac.block<3,3>(0,3) = delta_t* MatrixXd::Identity(3,3);
			f_jac.block<3,3>(3,6) =
							-skew_symetric(transform_imu * imu_f) * delta_t;


			// 2. Propagate uncertainty
			Matrix<double, 6, 6> q_cov = MatrixXd::Zero(6,6);
			q_cov.block<3,3>(0,0) =  MatrixXd::Identity(3,3)
															 *var_imu_f * pow(delta_t,2);
			q_cov.block<3,3>(3,3) =  MatrixXd::Identity(3,3)*
															 var_imu_w * pow(delta_t,2);


			p_cov = f_jac * p_cov * f_jac.transpose() +
							l_jac * q_cov * l_jac.transpose();

			p_est_pred = p_est; v_est_pred = v_est;
			q_est_pred = q_est;

			Matrix<double, 10, 1> state;
			state << p_est,v_est,q_est;
			std::cout << "prediction -> " << std::endl;
			return state;
		}

		Matrix<double, 10, 1> measurement_update(double var_sensor,
																						 Matrix<double, 3, 1> y){

			Matrix<double,3,3> R_cov = var_sensor * MatrixXd::Identity(3,3);
			Matrix<double, 9, 3> K= MatrixXd::Zero(9,3);
			K = p_cov * h_jac.transpose() *
					(h_jac* p_cov * h_jac.transpose()
					 + R_cov).inverse();

			Matrix<double, 9, 1> delta_x;
			//3.2 Compute error state
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
			Quaternion<double> q_est_obj(q_est(0),q_est(1),
																	 q_est(2),q_est(3));
			q_est_obj = q*q_est_obj;
			q_est_obj = q_est_obj.normalized();
			q_est << q_est_obj.w(),
							q_est_obj.x(),
							q_est_obj.y(),
							q_est_obj.z();
			//TODO add pcov i retun it
			p_cov = (MatrixXd::Identity(9,9) - K*h_jac) * p_cov;
			Matrix<double, 10, 1> state;
			state << p_est,v_est,q_est;

			std::cout << "measurement update -> " << std::endl;

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
		Matrix<double, 9, 9>	f_jac;
		Matrix<double, 3, 9>	h_jac;
		Matrix<double, 3, 1>	p_est_pred, v_est_pred, p_est,v_est;
		Matrix<double, 4, 1>	q_est_pred,q_est;
		Matrix<double, 9, 9>  p_cov;
		bool initialized;

};

#endif //SENSOR_FUSION_ES_EKF_HPP