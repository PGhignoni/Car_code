#ifndef ATTITUDEFILTER_H_INCLUDED
#define ATTITUDEFILTER_H_INCLUDED

#include "Attitude.h"
#include <eigen3/Eigen/Dense>


Eigen::Matrix3f crossMatrix(Eigen::Vector3f inputVector);


class AttitudeFilter{

    protected:

        //filter time step
        float m_dt;

        // Attitude estimate
	Attitude m_attitude;
	
        // angular speed estimate
        float m_omega[3];


        AttitudeFilter(float dt): m_dt{dt}, m_attitude{0.0,0.0,0.0,1.0}, m_omega{0.0,0.0,0.0}
        {
        }


};


class MEKF: public AttitudeFilter{


	protected:

	    Eigen::Vector3f m_dTheta_kp;			// attitude error (post-correction, i.e., after that the measurement is given)
	    Eigen::Vector3f m_deltaBeta_kp;			// correction of the accelerometers biases (post-correction)
	    Eigen::Vector3f m_beta_kp;				// estimate of the accelerometer bias (post-correction)
	    Eigen::Vector3f m_beta_km;				// estimate of the accelerometer bias at previous time step (post-prediction)
	    Eigen::Matrix<float, 6, 1> m_deltaX_kp;		// correction of the filter states (post-correction)
	    Eigen::Vector3f m_omega_kp;				// estimate of the angular speed at current time step (post-correction)
	    Eigen::Vector3f m_omega_km;				// estimate of the angular speed at previous time step (post-prediction)
	    Eigen::Matrix<float, 3, 3> m_R_k;			// measurement noise covariance matrix					
	    Eigen::Matrix<float, 6, 6> m_Q_k;			// process noise covariance matrix
	    Eigen::Vector4f m_q_kp;				// quaternion estimate at current time step (post-correction)
	    Eigen::Vector4f m_q_km;				// quaternion estimate at previous time step (post-prediction)
	    Eigen::Matrix<float, 6, 6> m_P_kp;			// state covariance (post-correction)
	    Eigen::Matrix<float, 6, 6> m_P_km;			// state covariance before correction (after prediction)
	    Eigen::Matrix<float, 3, 6> m_H_k;			// Kalman measurement matrix
	    Eigen::Vector3f m_h_k;				// observation (post-prediction)
	    Eigen::Matrix<float, 6, 3> m_K_k;			// Kalman gain	
	    Eigen::Matrix<float, 3, 3> m_A_q_k;			// Attitude matrix (post-prediction)




        public:

            MEKF(float dt, float sigma_RRW, float sigma_ARW, float sigma_accel): AttitudeFilter{dt} 
            {
		m_dTheta_kp<< 0.0,0.0,0.0;
		m_deltaBeta_kp<< 0.0,0.0,0.0;
		m_beta_km << 0.0,0.0,0.0;
		m_R_k=Eigen::Matrix<float, 3, 3>::Identity()*sigma_accel*sigma_accel;
		m_Q_k.block<3,3>(0,0)=Eigen::Matrix3f::Identity()*sigma_ARW*sigma_ARW;
		m_Q_k.block<3,3>(3,0)=Eigen::Matrix3f::Zero();
		m_Q_k.block<3,3>(0,3)=Eigen::Matrix3f::Zero();
		m_Q_k.block<3,3>(3,3)=Eigen::Matrix3f::Identity()*sigma_RRW*sigma_RRW;
		m_q_km<< 0.0,0.0,0.0,1.0;
		m_P_km=Eigen::Matrix<float, 6, 6>::Zero();
		m_A_q_k=Eigen::Matrix3f::Identity();
            }
    
	    // prints the states of the filter
	    void displayFilterStates();
		    
	    // KF prediction
	    void filterPredict(float omega_x, float omega_y, float omega_z);

	    // KF correction given the accelerometer measurement	
	    void filterCorrectAccelerometer(float a_x, float a_y, float a_z);

	    // Computation of the optimal observer gain
	    void filterGainComputation();


};

#endif // ATTITUDEFILTER_H_INCLUDED
