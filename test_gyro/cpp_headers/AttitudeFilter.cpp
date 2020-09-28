#include "AttitudeFilter.h"
#include "Attitude.h"
#include <eigen3/Eigen/Dense>
#include <iostream>

const Eigen::Vector3f gravityAcceleration{0.0,0.0,9.81};

Eigen::Matrix3f crossMatrix(Eigen::Vector3f in){

	Eigen::Matrix3f outputMatrix;
	outputMatrix<< 0., -in(2), in(1), in(2), 0., -in(0), -in(1), in(0), 0.;
	return outputMatrix;

};



void MEKF::displayFilterStates(){

	float euler[3];
	m_attitude.getEuler(euler);
	std::cout<<"Printing the Euler angles"<<std::endl;
	std::cout<<"phi="<< euler[0]<<std::endl;
	std::cout<<"theta="<< euler[1] <<std::endl;
	std::cout<<"psi="<< euler[2] <<std::endl;
	std::cout<<std::endl;
	std::cout<<"Printing the accelerometer biases"<<std::endl;
	std::cout<<"beta1="<< m_beta_kp(0)<<std::endl;
	std::cout<<"beta2="<< m_beta_kp(1) <<std::endl;
	std::cout<<"beta3="<< m_beta_kp(2) <<std::endl;
	std::cout<<std::endl;

};


void MEKF::filterPredict(float omega_x, float omega_y, float omega_z){

	// estimate the angular speed
	Eigen::Vector3f omega_meas{omega_x, omega_y, omega_z};
	m_omega_kp=omega_meas-m_beta_kp; 
	// update the angular velocity	
	m_omega[0]=m_omega_kp(0);
	m_omega[1]=m_omega_kp(1);
	m_omega[2]=m_omega_kp(2);

	// covariance propagation
	Eigen::Matrix<float, 6, 6> F_k;
	F_k<<-1*crossMatrix(m_omega_kp), -1*Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(); 
	Eigen::Matrix<float, 6, 6> G_k;
	G_k<< -1*Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(), -1*Eigen::Matrix3f::Identity();	
	m_P_km=m_P_kp+m_dt*(F_k*m_P_kp+m_P_km*F_k.transpose()+G_k*m_Q_k*G_k.transpose());		// Riccati equation 

	// attitude propagation
	Eigen::Vector3f sigma{(m_omega_kp+m_omega_km)/2*m_dt};
	Eigen::Matrix4f transitionMatrix;	
	transitionMatrix.block<3,3>(0,0)=cos(sigma.norm()/2)*Eigen::Matrix3f::Identity()-sin(sigma.norm()/2)/2*crossMatrix(sigma);
	transitionMatrix.block<3,1>(0,3)=sin(sigma.norm()/2)/2*sigma;
	transitionMatrix.block<1,3>(3,0)=sin(sigma.norm()/2)/2*sigma.transpose();
	transitionMatrix(3,3)=cos(sigma.norm()/2);
	m_q_km=transitionMatrix*m_q_kp;									// cumbersome notation, see the definitions
	m_beta_km=m_beta_kp;
	m_omega_km=m_omega_kp;

	// update the attitude of the filter after prediction
	m_attitude.updateAttitude(m_q_km(0),m_q_km(1),m_q_km(2),m_q_km(3));
	m_attitude.getAttitudeMatrix(m_A_q_k);
	
	
};



void MEKF::filterCorrectAccelerometer(float a_x, float a_y, float a_z){


	m_h_k=m_A_q_k*gravityAcceleration;
	Eigen::Vector3f y_k{ a_x,a_y,a_z};
	m_P_kp=(Eigen::Matrix<float, 6, 6>::Identity()-m_K_k*m_H_k)*m_P_km;
	m_deltaX_kp=m_K_k*(y_k-m_h_k);
	m_dTheta_kp=m_deltaX_kp.head(3);
	m_deltaBeta_kp=m_deltaX_kp.tail(3);
	Eigen::Vector4f q_star;
	Eigen::Matrix<float, 4, 3> Theta;
	Theta<<m_q_km(4)*Eigen::Matrix3f::Identity()+crossMatrix(m_q_km.head(3)), -m_q_km.head(3).transpose();
	q_star=m_q_km+0.5*Theta*m_dTheta_kp;
	m_q_kp=q_star/q_star.norm();
	m_beta_kp=m_beta_km+m_deltaBeta_kp;	
	// update the attitude of the filter after correction
	m_attitude.updateAttitude(m_q_kp(0),m_q_kp(1),m_q_kp(2),m_q_kp(3));


};



void MEKF::filterGainComputation(){
	
	m_H_k.block<3,3>(0,0)= m_A_q_k*crossMatrix(gravityAcceleration);
	m_H_k.block<3,3>(0,3)= Eigen::Matrix3f::Zero();	
	Eigen::Matrix3f tmp;
	tmp<<m_H_k*m_P_km*m_H_k.transpose()+m_R_k;
	m_K_k=m_P_km*m_H_k.transpose()*tmp.inverse();							// Kalman gain

};


