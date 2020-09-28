#include <stlport.h>
#include <Eigen30.h>
#include "AttitudeFilter.h"
#include <iostream>

const Eigen::Vector3f gravityAcceleration{0.0,0.0,1.};

Eigen::Matrix3f crossMatrix(Eigen::Vector3f in){

	Eigen::Matrix3f outputMatrix;
	outputMatrix<< 0., -in(2), in(1), in(2), 0., -in(0), -in(1), in(0), 0.;
	return outputMatrix;

};



void MEKF::filterPredict(float omega_x, float omega_y, float omega_z){

	// estimate the angular speed
	Eigen::Vector3f omega_meas{omega_x, omega_y, omega_z};
	Eigen::Vector3f sigma{(omega_meas-m_beta+m_omega)/2*m_dt};			// vector for the attitude propagation (if implemented here saves memory)
	m_omega=omega_meas-m_beta; 


	// covariance propagation
	Eigen::Matrix<float, 6, 6> F_k;
	F_k<<-1*crossMatrix(m_omega), -1*Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(); 
	Eigen::Matrix<float, 6, 6> G_k;
	G_k<< -1*Eigen::Matrix3f::Identity(), Eigen::Matrix3f::Zero(), Eigen::Matrix3f::Zero(), -1*Eigen::Matrix3f::Identity();	
	m_P=m_P+m_dt*(F_k*m_P+m_P*F_k.transpose()+G_k*m_Q*G_k.transpose());		// Riccati equation 

	// attitude propagation
	Eigen::Matrix4f transitionMatrix;	
	transitionMatrix.block<3,3>(0,0)=cos(sigma.norm()/2)*Eigen::Matrix3f::Identity()-sin(sigma.norm()/2)/2*crossMatrix(sigma);
	transitionMatrix.block<3,1>(0,3)=sin(sigma.norm()/2)/2*sigma;
	transitionMatrix.block<1,3>(3,0)=sin(sigma.norm()/2)/2*sigma.transpose();
	transitionMatrix(3,3)=cos(sigma.norm()/2);
	m_q=transitionMatrix*m_q;									

	// update the attitude of the filter after prediction 
	MEKF::updateAttitudeMatrix();
	
	
};



void MEKF::filterCorrectAccelerometer(float a_x, float a_y, float a_z){


	m_h=m_A_q*gravityAcceleration;
	Eigen::Vector3f y{ a_x,a_y,a_z};
	m_P=(Eigen::Matrix<float, 6, 6>::Identity()-m_K*m_H)*m_P;
	m_deltaX=m_K*(y-m_h);
	m_dTheta=m_deltaX.head(3);
	m_deltaBeta=m_deltaX.tail(3);
	//Eigen::Vector4f q_star;
	Eigen::Matrix<float, 4, 3> Theta;
	Theta<<m_q(4)*Eigen::Matrix3f::Identity()+crossMatrix(m_q.head(3)), -m_q.head(3).transpose();
	//q_star=m_q_km+0.5*Theta*m_dTheta_kp;
	//m_q_kp=q_star/q_star.norm();
	m_q=m_q+0.5*Theta*m_dTheta;
	m_q=m_q/m_q.norm();
	m_beta=m_beta+m_deltaBeta;
	
	// update the attitude of the filter after correction
	MEKF::updateAttitudeMatrix();


};



void MEKF::filterGainComputation(){
	
	m_H.block<3,3>(0,0)= m_A_q*crossMatrix(gravityAcceleration);
	m_H.block<3,3>(0,3)= Eigen::Matrix3f::Zero();	
	Eigen::Matrix3f tmp;
	tmp<<m_H*m_P*m_H.transpose()+m_R;
//  Eigen::Matrix3f tmp_inverse;
//  Eigen::FullPivLU<Eigen::Matrix3f> lu{tmp};//(tmp);
//  tmp_inverse=tmp.inverse();
//  tmp=tmp.colPivHouseholderQr().solve(Eigen::Matrix3f::Identity());
//  Eigen::Matrix3f inverseMatrix = tmp.lu()<Eigen::Matrix3f>.solve(Eigen::Matrix3f::Identity());
//  Eigen::internal::inverse_impl<Eigen::Matrix<float, 3, 3> > GG;
	m_K=m_P*m_H.transpose()*tmp.inverse();							// Kalman gain

};


void MEKF::updateAttitudeMatrix(){
	float q1{m_q(0)};
	float q2{m_q(1)};
	float q3{m_q(2)};
	float q4{m_q(3)};		
	m_A_q(0,0)=q1*q1-q2*q2-q3*q3+q4*q4;
        m_A_q(0,1)=2*(q1*q2+q3*q4);
        m_A_q(0,2)=2*(q1*q3-q2*q4);
        m_A_q(1,0)=2*(q2*q1-q3*q4);
        m_A_q(1,1)=-q1*q1+q2*q2-q3*q3+q4*q4;
        m_A_q(1,2)= 2*(q2*q3+q1*q4);
        m_A_q(2,0)=2*(q3*q1+q2*q4);
        m_A_q(2,1)=2*(q3*q2-q1*q4);
        m_A_q(2,2)=-q1*q1-q2*q2+q3*q3+q4*q4;
};


void MEKF::getEulerAngles(float euler[3]){
  
  	euler[1]=-asin(m_A_q(2,0));                  // theta
        euler[0]=atan(m_A_q(2,1)/m_A_q(2,2));        // phi
        euler[2]=atan(m_A_q(1,0)/m_A_q(0,0));        // psi
  
};
