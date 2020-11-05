#ifndef SENSOR_FUSION_ES_EKF_HPP
#define SENSOR_FUSION_ES_EKF_HPP

#include <Eigen/Geometry>
#include "utils.hpp"
using namespace Eigen;
class EsEkf{
public:
		EsEkf(){
			g<< 0,0,-9.81;
			l_jac=  MatrixXd::Zero(15, 12);  // motion model noise jacobian
			l_jac.bottomLeftCorner(12,12) =
							MatrixXd::Identity(12,12);

			h_jac;  //measurement model jacobian
			h_jac =  MatrixXd::Zero(3,15);
			h_jac.bottomLeftCorner(3,3) = MatrixXd::Identity(3,3);

			p_est << 0, 0,0;
			v_est << 0,0,0;
			q_est << 1,0,0,0;
			fb_est << 0,0,0; wb_est << 0,0,0;
			p_cov = MatrixXd::Zero(15,15);
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

		bool isIinit(){return initialized;}

		Matrix<double, 10, 1> predicition(
						Matrix<double, 3, 1> imu_f,
						Matrix<double, 3,3> var_imu_f,
						Matrix<double, 3, 1> imu_w,
						Matrix<double, 3,3> var_imu_w,
						double delta_t){
			initialized = true;
			Quaternion<double> q(q_est(0),q_est(1),
													 q_est(2),q_est(3));
			// 1. Update state with IMU inputs
			Matrix3d transform_imu = q.toRotationMatrix();
			p_est = p_est + delta_t*v_est +
							pow(delta_t,2)/2 * (transform_imu * (imu_f-fb_est) + g);
			v_est = v_est + delta_t * (transform_imu * (imu_f-fb_est) + g);

			Quaternion<double> imu_q = euler2quat({delta_t*(imu_w[0]-wb_est[0]),
																						 delta_t*(imu_w[1]-wb_est[1]),
																						 delta_t*(imu_w[2]-wb_est[2])});


			fb_est = fb_est; wb_est = wb_est;
			//1.1 Linearize the motion model and compute Jacobians
			f_jac = MatrixXd::Identity(15,15);
			f_jac.block<3,3>(0,3) = delta_t* MatrixXd::Identity(3,3);
			f_jac.block<3,3>(3,6) =
							-skew_symetric(transform_imu * (imu_f - fb_est)) * delta_t;
			f_jac.block<3,3>(3,9) = -transform_imu * delta_t;
			f_jac.block<3,3>(6,12) = -delta_t * MatrixXd::Identity(3,3);
			f_jac.block<3,3>(6,6) = (imu_q.toRotationMatrix()).transpose();

			imu_q = q*imu_q;
			imu_q = imu_q.normalized();
			q_est << imu_q.w(),imu_q.x(),imu_q.y(),imu_q.z();

			// 2. Propagate uncertainty
			double var_imu_fb = 0;
			double var_imu_wb = 0;
			Matrix<double, 12, 12> q_cov = MatrixXd::Identity(12,12);
			q_cov.block<3,3>(0,0) =  var_imu_f * pow(delta_t,2);
			q_cov.block<3,3>(3,3) =  var_imu_w * pow(delta_t,2);
			q_cov.block<3,3>(6,6) =  MatrixXd::Identity(3,3)*
															 var_imu_fb * delta_t;
			q_cov.block<3,3>(9,9) =  MatrixXd::Identity(3,3)*
															 var_imu_wb * delta_t;


			p_cov = f_jac * p_cov * f_jac.transpose() +
							l_jac * q_cov * l_jac.transpose();

			p_est_pred = p_est; v_est_pred = v_est;
			q_est_pred = q_est;

			Matrix<double, 10, 1> state;
			state << p_est,v_est,q_est;
			std::cout << " Prediction   update -> " << p_est.transpose() << std::endl;
			return state;
		}

		Matrix<double, 10, 1> measurement_update(Matrix<double, 3,3> var_sensor,
																						 Matrix<double, 3, 1> y){

			Matrix<double,3,3> R_cov = var_sensor;
			Matrix<double, 15, 3> K= MatrixXd::Zero(15,3);
			K = p_cov * h_jac.transpose() *
					(h_jac* p_cov * h_jac.transpose()
					 + R_cov).inverse();
			Matrix<double, 15, 1> delta_x;
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

			fb_est = fb_est +  delta_x.block<3,1>(9,0);
			wb_est = wb_est +  delta_x.block<3,1>(12,0);
			p_cov = (MatrixXd::Identity(15,15) - K*h_jac)
							* p_cov *
							(MatrixXd::Identity(15,15) - K*h_jac).transpose() +
							K*R_cov*K.transpose();
			Matrix<double, 10, 1> state;
			state << p_est,v_est,q_est;
			std::cout << " Measurement update -> " << p_est.transpose() << std::endl;
			return state;
		}

		Matrix<double, 10, 1> angle_measurement_update(Matrix<double, 3,3> var_sensor,
																						 Matrix<double, 4, 1> y){

			Matrix<double,3,3> R_cov = var_sensor;
			Matrix<double, 15, 3> K= MatrixXd::Zero(15,3);
			Matrix<double, 3, 15> h_jac_angle =MatrixXd::Zero(3,15);
			h_jac_angle(0,6) = 1; //quat -> w
			h_jac_angle(1,7) = 1; //quat -> x
			h_jac_angle(2,8) = 1; //quat -> y

			K = p_cov * h_jac_angle.transpose() *
					(h_jac_angle* p_cov * h_jac_angle.transpose()
					 + R_cov).inverse();
			Matrix<double, 15, 1> delta_x;
			//3.2 Compute error state
			Quaternion<double> q_est_obj(q_est(0),q_est(1),
																	 q_est(2),q_est(3));
			Quaternion<double> delta_quat(y[0],y[1],y[2],y[3]);
			delta_quat.normalize();
			q_est_obj.normalize();
			delta_quat = q_est_obj.inverse()*delta_quat;
			delta_quat.normalize();

			AngleAxis<double> angle_axis(delta_quat.toRotationMatrix());;
			delta_x = K * (angle_axis.angle()*angle_axis.axis());

			// 3.3 Correct predicted state
			p_est = p_est + delta_x.block<3,1>(0,0);
			v_est = v_est + delta_x.block<3,1>(3,0);
			Matrix<double,4,1> quat_from_aa = axixs_angle2quat(
							delta_x.block<3,1>(6,0));

			Quaternion<double> q(quat_from_aa(0),
													 quat_from_aa(1),
													 quat_from_aa(2),
													 quat_from_aa(3));
			q_est_obj = q*q_est_obj;
			q_est_obj = q_est_obj.normalized();
			q_est << q_est_obj.w(),
							q_est_obj.x(),
							q_est_obj.y(),
							q_est_obj.z();

			//TODO add pcov i retun it

			fb_est = fb_est +  delta_x.block<3,1>(9,0);
			wb_est = wb_est +  delta_x.block<3,1>(12,0);
			p_cov = (MatrixXd::Identity(15,15) - K*h_jac_angle)
							* p_cov *
							(MatrixXd::Identity(15,15) - K*h_jac_angle).transpose() +
							K*R_cov*K.transpose();
			Matrix<double, 10, 1> state;
			state << p_est,v_est,q_est;
			return state;
		}


		Matrix<double,10,1 > getState(){
			Matrix<double,10,1> state;
			state << p_est,v_est,q_est;
			return state;
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

private:
		// error state Ekf
		Matrix<double, 3, 1>  g;
		Matrix<double, 15, 12>	l_jac;
		Matrix<double, 15, 15>	f_jac;
		Matrix<double, 3, 15>	h_jac;
		Matrix<double, 3, 1>	p_est_pred, v_est_pred, p_est,v_est,fb_est,wb_est;
		Matrix<double, 4, 1>	q_est_pred,q_est;
		Matrix<double,15, 15>  p_cov;
		bool initialized;

};

#endif //SENSOR_FUSION_ES_EKF_HPP