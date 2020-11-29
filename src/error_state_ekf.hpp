#ifndef SENSOR_FUSION_ES_EKF_HPP
#define SENSOR_FUSION_ES_EKF_HPP

#include <iostream>
#include <Eigen/Geometry>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"

#include "utils.h"
#include "structures.h"

/*
 * Error state Kalman filter implementation based on:
 * Quaternion kinematics for the error-state Kalman filter
 * by Joan Sola -> pdf: https://arxiv.org/pdf/1711.02508.pdf
 * Read for more detail.
 *
 * We use imu acceleration and angular velocity measurements as
 * inputs for the prediction model.
 * For the update model we use position measurements and correct
 * the predicted state with the errors.
 * For the code to work properly it is expected that imu measurements
 * have at least a 3-4 times higher rate than the sensors used for
 * pose measurements (eg. imu: 100hz and sensor: 25 hz )
 */


using namespace Eigen;
class EsEkf{
public:
		EsEkf():initialized(false){
			// if the sensor are not initialized wait
			// dont call measurement update before the first prediction

			// gravity vector
			g<< 0,0,-10.34;

			// motion model noise jacobian
			l_jac=  MatrixXd::Zero(18, 12);
			l_jac.bottomLeftCorner(12,12) =
							MatrixXd::Identity(12,12);

			// initial state for pose, linear velocity and orientation
			p_est << 0, 0,0; v_est << 0,0,0; q_est << -1,0,0,0;
			// initial state for imu accelerometer and gyroscope bias
			fb_est << 0,0,0; wb_est << 0,0,0;
			// initital state error is zero (can be anything else)
			p_cov = MatrixXd::Identity(18,18);
			//p_cov.bottomRightCorner(6,6) =MatrixXd::Zero(6,6);
			std::cout << "p_ccov-> \n" << p_cov;
			ROS_INFO("Es EKF init %d",initialized);
			camera_drift << 0,0,0;
		}

		void setInit(){initialized = true;}
		bool isInit(){return initialized;}


		/*
		 * The prediction step. Here we use imu measurements to get an
		 * estimate of the pose, velocity and orientation.
		 */
		void prediction(
						Matrix<double, 3, 1> imu_f,
						Matrix<double, 3,3> var_imu_f,
						Matrix<double, 3, 1> imu_w,
						Matrix<double, 3,3> var_imu_w,
						double delta_t){
			Quaternion<double> q(q_est(0),q_est(1),
													 q_est(2),q_est(3));

			// 1. Update state with IMU inputs
			Matrix3d transform_imu = q.toRotationMatrix();
			p_est = p_est + delta_t*v_est +
							pow(delta_t,2)/2 * (transform_imu * (imu_f-fb_est) + g);

//			std::cout << "p_est-> " << p_est.transpose() << '\n'
//								<< "v_est-> " << v_est.transpose() << '\n'
//								<< "pow(delta_t,2)/2 * (transform_imu * (imu_f-fb_est) + g);"
//								<< (pow(delta_t,2)/2 * (transform_imu * (imu_f-fb_est) + g)).transpose() << '\n';

			v_est = v_est + delta_t * (transform_imu * (imu_f-fb_est) + g);

			Quaternion<double> imu_q = euler2quat({delta_t*(imu_w[0]-wb_est[0]),
																						 delta_t*(imu_w[1]-wb_est[1]),
																						 delta_t*(imu_w[2]-wb_est[2])});

			fb_est = fb_est; wb_est = wb_est; //bias is constant
//			ROS_INFO("info of fb_est %f %f",fb_est[1],wb_est[1]);
			//1.1 Linearize the motion model and compute Jacobians (Scola (270))
			f_jac = MatrixXd::Identity(18,18);
			f_jac.block<3,3>(0,3) = delta_t* MatrixXd::Identity(3,3);
			f_jac.block<3,3>(3,6) =
							-skew_symetric(transform_imu * (imu_f - fb_est)) * delta_t;
			f_jac.block<3,3>(3,9) = -transform_imu * delta_t;
			f_jac.block<3,3>(6,12) = -delta_t * MatrixXd::Identity(3,3);
			f_jac.block<3,3>(6,6) = (imu_q.toRotationMatrix()).transpose();

			//Propagate orientation
			imu_q = q*imu_q;
			imu_q.normalize();
			q_est << imu_q.w(),imu_q.x(),imu_q.y(),imu_q.z();

			// 2. Propagate uncertainty
			double var_imu_fb = 0.0;
			double var_imu_wb = 0.0;
			Matrix<double, 12, 12> q_cov = MatrixXd::Identity(12,12);
			q_cov.block<3,3>(0,0) =  var_imu_f * pow(delta_t,2);
			q_cov.block<3,3>(3,3) =  var_imu_w * pow(delta_t,2);
			q_cov.block<3,3>(6,6) =  MatrixXd::Identity(3,3)*
															 var_imu_fb * delta_t;
			q_cov.block<3,3>(9,9) =  MatrixXd::Identity(3,3)*
															 var_imu_wb * delta_t;


			p_cov = f_jac * p_cov * f_jac.transpose() +
							l_jac * q_cov * l_jac.transpose();
			ROS_INFO("Prediction");

		}


		/*
		 * Measurement update. Use sensor measurement of pose to
		 * update the estimation and reduce uncertainty.
		 *
		 */
		void measurement_update(Matrix<double, 3,3> R_cov,
														Matrix<double, 3, 1> y){
			//measurement model jacobian
			Matrix<double, 3, 18>	h_jac =  MatrixXd::Zero(3,18);
			h_jac.bottomLeftCorner(3,3) = MatrixXd::Identity(3,3);
			//K -> Kalman gain
			Matrix<double, 18, 3> K= MatrixXd::Zero(18,3);
			K = p_cov * h_jac.transpose() *
					(h_jac* p_cov * h_jac.transpose()
					 + R_cov).inverse();

			// delta_x -> Error state
			Matrix<double, 18, 1> delta_x;
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

			fb_est = fb_est +  delta_x.block<3,1>(9,0);
			wb_est = wb_est +  delta_x.block<3,1>(12,0);
			camera_drift = camera_drift + delta_x.block<3,1>(15,0);
			p_cov = (MatrixXd::Identity(18,18) - K*h_jac)
							* p_cov *
							(MatrixXd::Identity(18,18) - K*h_jac).transpose() +
							K*R_cov*K.transpose();
//			std::cout << "fb_est-> " << fb_est.transpose() << '\n'
//								<< "wb_est-> " << wb_est.transpose() << '\n';
			ROS_INFO("Pose measurement");

		}

		void camera_measurement_update(Matrix<double, 3,3> R_cov,
																   Matrix<double, 3, 1> y){
			//measurement model jacobian
			Matrix<double, 3, 18>	h_jac =  MatrixXd::Zero(3,18);
			h_jac.bottomLeftCorner(3,3) = MatrixXd::Identity(3,3);
			h_jac.bottomRightCorner(3,3) = MatrixXd::Identity(3,3);
			//K -> Kalman gain
			Matrix<double, 18, 3> K= MatrixXd::Zero(18,3);
			K = p_cov * h_jac.transpose() *
					(h_jac* p_cov * h_jac.transpose()
					 + R_cov).inverse();

			// delta_x -> Error state
			Matrix<double, 18, 1> delta_x;
			//3.2 Compute error state
			delta_x = K * (y - (p_est+camera_drift));
			std::cout << "Delta pose ------>" << (y - (p_est+camera_drift)).transpose() <<  '\n';
			std::cout << "Delta pose old -->" << (y - p_est).transpose() <<  '\n';
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

			fb_est = fb_est +  delta_x.block<3,1>(9,0);
			wb_est = wb_est +  delta_x.block<3,1>(12,0);
			camera_drift = camera_drift + delta_x.block<3,1>(15,0);
			std::cout << "Camera Drift -----------------> " << camera_drift.transpose() << '\n';
			p_cov = (MatrixXd::Identity(18,18) - K*h_jac)
							* p_cov *
							(MatrixXd::Identity(18,18) - K*h_jac).transpose() +
							K*R_cov*K.transpose();
//			std::cout << "fb_est-> " << fb_est.transpose() << '\n'
//								<< "wb_est-> " << wb_est.transpose() << '\n';
			ROS_INFO("Pose measurement");

		}




		Matrix<double, 10, 1> angle_measurement_update(Matrix<double, 3,3> var_sensor,
																									 Matrix<double, 4, 1> y){



			Matrix<double,3,3> R_cov = var_sensor;
			Matrix<double, 18, 3> K= MatrixXd::Zero(18,3);
			Matrix<double, 3, 18> h_jac_angle = MatrixXd::Zero(3,18);
			h_jac_angle(0,6) = 1; //quat -> w
			h_jac_angle(1,7) = 1; //quat -> x
			h_jac_angle(2,8) = 1; //quat -> y


			K = p_cov * h_jac_angle.transpose() *
					(h_jac_angle* p_cov * h_jac_angle.transpose()
					 + R_cov).inverse();
			Matrix<double, 18, 1> delta_x;
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
			camera_drift = camera_drift + delta_x.block<3,1>(15,0);
			p_cov = (MatrixXd::Identity(18,18) - K*h_jac_angle)
							* p_cov *
							(MatrixXd::Identity(18,18) - K*h_jac_angle).transpose() +
							K*R_cov*K.transpose();
			Matrix<double, 10, 1> state;
			state << p_est,v_est,q_est;
			ROS_INFO("Angle measurement %f",angle_axis.angle());
			return state;
		}


		void velocity_measurement_update(Matrix<double, 3,3> R_cov,
																		 Matrix<double, 3, 1> y){
			//measurement model jacobian
			Matrix<double, 3, 18>	h_jac =  MatrixXd::Zero(3,18);
			h_jac(0,3) = 1;
			h_jac(1,4) = 1;
			h_jac(2,5) = 1;
			//K -> Kalman gain
			Matrix<double, 18, 3> K= MatrixXd::Zero(18,3);
			K = p_cov * h_jac.transpose() *
					(h_jac* p_cov * h_jac.transpose()
					 + R_cov).inverse();

			// delta_x -> Error state
			Matrix<double, 18, 1> delta_x;
			//3.2 Compute error state
			delta_x = K * (y - v_est);

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

			fb_est = fb_est +  delta_x.block<3,1>(9,0);
			wb_est = wb_est +  delta_x.block<3,1>(12,0);
			camera_drift = camera_drift + delta_x.block<3,1>(15,0);
			p_cov = (MatrixXd::Identity(18,18) - K*h_jac)
							* p_cov *
							(MatrixXd::Identity(18,18) - K*h_jac).transpose() +
							K*R_cov*K.transpose();

			ROS_INFO("Velocity measurement");

		}


		Matrix<double,3,1> getPose(){return p_est;}
		Matrix<double,4,1> getOrientation(){return q_est;}
		Matrix<double,10,1 > getState(){
			Matrix<double,10,1> state;
			state << p_est,v_est,q_est;
			return state;
		}

		void setPose(Matrix<double, 3, 1> initial_state){
			p_est << initial_state(0),
							initial_state(1),
							initial_state(2);
			std::cout << "p_est " << p_est << std::endl;
		}
		void setVel(Matrix<double, 3, 1> initial_state){
			v_est << initial_state(0),
							initial_state(1),
							initial_state(2);
			std::cout << "v_est " << v_est << std::endl;
		}
		void setQuat(Matrix<double, 4, 1> initial_state){
			q_est << initial_state(0),
							initial_state(1),
							initial_state(2),
							initial_state(3);
			std::cout << "Q_est " << q_est << std::endl;
		}

private:

		/*
		 * ES-EKF matrices definition
		 */
		// Gravity vector
		Matrix<double, 3, 1>    g;
		// motion model noise jacobian
		Matrix<double, 18, 12>	l_jac;
		// Linearized motion model Jacobian
		Matrix<double, 18, 18>	f_jac;
		//States we estamte
		Matrix<double, 3, 1>	  p_est,v_est,fb_est,wb_est,camera_drift;
		Matrix<double, 4, 1>	  q_est;
		// Covariance matrix
		Matrix<double,18, 18>   p_cov;
		bool initialized;
};

#endif //SENSOR_FUSION_ES_EKF_HPP