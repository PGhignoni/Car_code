#ifndef ATTITUDE_H_INCLUDED
#define ATTITUDE_H_INCLUDED

#include <eigen3/Eigen/Dense>

class Attitude{

    protected:

        float m_q[4];                     // unit quaternion in the form [vectorial, scalar]

        float m_euler[3];                 // Euler angles in the sequence [phi, theta, psi]

        //float m_attitudeMatrix[3][3];   // rotation matrix

	Eigen::Matrix3f m_attitudeMatrix; // attitude matrix

        float m_rotationVector[3];        // rotation vector

        float m_rotationAngle;            // angle of rotation

        // functions for the conversion of the orientation
        void quatToEuler();
        void quatToAtt();
        void quatToRodr();
        void eulerToAtt();
        void attToQuat();
        void rodrToQuat();

    public:

        // constructors
        Attitude(const float q1, const float q2, const float q3, const float q4);

        Attitude(const float phi,const float theta,const float psi);

        Attitude(const float attitudeMatrix[3][3]);

        Attitude(const float rotationVector[3], const float rotationAngle);

        // methods to write the attitude 

        void getQuaternion(float quaternion[4]);

        void getEuler(float euler[3]);

        void getAttitudeMatrix(float attitudeMatrix[3][3]);

        void getRotationParams(float rotationVector[3],float &rotationAngle);

        // methods to update the attitude (TODO)

        void updateAttitude(const float q1, const float q2, const float q3, const float q4);
        void updateAttitude(const float phi,const float theta,const float psi);
        void updateAttitude(const float attitudeMatrix[3][3]);
        void updateAttitude(const float rotationVector[3], const float rotationAngle);

	// method to print attitude information

	void displayAttitude();



};



#endif // ATTITUDE_H_INCLUDED
