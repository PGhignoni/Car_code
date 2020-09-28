#ifndef ATTITUDEFILTER_H_INCLUDED
#define ATTITUDEFILTER_H_INCLUDED
#include <stlport.h>
#include <Eigen30.h>


Eigen::Matrix3f crossMatrix(Eigen::Vector3f inputVector);


class AttitudeFilter{

    protected:

        //filter time step
        float m_dt;

        // Attitude estimate
	Eigen::Matrix<float, 4, 1> m_q;
	
        // angular speed estimate
        Eigen::Matrix<float, 3, 1> m_omega;


        AttitudeFilter(float dt): m_dt{dt}, m_q{0.0,0.0,0.0,1.0}, m_omega{0.0,0.0,0.0}
        {
        }


};


class MEKF: public AttitudeFilter{


	protected:

	    Eigen::Vector3f m_dTheta;			// attitude error 
	    Eigen::Vector3f m_deltaBeta;		// correction of the accelerometers biases 
	    Eigen::Vector3f m_beta;			// estimate of the accelerometer bias (post-correction)
	    Eigen::Matrix<float, 6, 1> m_deltaX;	// correction of the filter states 
	    Eigen::Matrix<float, 3, 3> m_R;		// measurement noise covariance matrix					
	    Eigen::Matrix<float, 6, 6> m_Q;		// process noise covariance matrix
	    Eigen::Matrix<float, 6, 6> m_P;		// state covariance 
	    Eigen::Matrix<float, 3, 6> m_H;		// Kalman measurement matrix
	    Eigen::Vector3f m_h;			// observation 
	    Eigen::Matrix<float, 6, 3> m_K;		// Kalman gain	
	    Eigen::Matrix<float, 3, 3> m_A_q;		// Attitude matrix (post-prediction)




        public:

            MEKF(float dt, float sigma_RRW, float sigma_ARW, float sigma_accel): AttitudeFilter{dt} 
            {
		m_dTheta<< 0.0,0.0,0.0;
		m_deltaBeta<< 0.0,0.0,0.0;
		m_beta << 0.0,0.0,0.0;
		m_R=Eigen::Matrix<float, 3, 3>::Identity()*sigma_accel*sigma_accel;
		m_Q.block<3,3>(0,0)=Eigen::Matrix3f::Identity()*sigma_ARW*sigma_ARW;
		m_Q.block<3,3>(3,0)=Eigen::Matrix3f::Zero();
		m_Q.block<3,3>(0,3)=Eigen::Matrix3f::Zero();
		m_Q.block<3,3>(3,3)=Eigen::Matrix3f::Identity()*sigma_RRW*sigma_RRW;
		m_P=Eigen::Matrix<float, 6, 6>::Zero();
		m_A_q=Eigen::Matrix3f::Identity();
            }
    
		    
	    // KF prediction
	    void filterPredict(float omega_x, float omega_y, float omega_z);

	    // KF correction given the accelerometer measurement	
	    void filterCorrectAccelerometer(float a_x, float a_y, float a_z);

	    // Computation of the optimal observer gain
	    void filterGainComputation();

	    // update the attitude matrix
	    void updateAttitudeMatrix();

     	    // obtain the Euler angles
            void getEulerAngles(float euler[3]);


};

#endif // ATTITUDEFILTER_H_INCLUDED
