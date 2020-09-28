#include "Attitude.h"
#include <stlport.h>
#include <math.h>
#include <iostream>
#include <Eigen30.h>

// constructors implementation

Attitude::Attitude(const float q1, const float q2, const float q3, const float q4){

        m_q[0]=q1;
        m_q[1]=q2;
        m_q[2]=q3;
        m_q[3]=q4;

        Attitude::quatToEuler();    // it calls quatToAtt!
        Attitude::quatToRodr();
};

Attitude::Attitude(const float phi,const float theta,const float psi){

        m_euler[0]=phi;
        m_euler[1]=theta;
        m_euler[2]=psi;

        Attitude::eulerToAtt();
        Attitude::attToQuat();
        Attitude::quatToRodr();
};

Attitude::Attitude(const float attitudeMatrix[3][3]){

        for (int i{0};i<=2;i++){
            for (int j{0};j<=2;j++){
                //m_attitudeMatrix[i][j]=attitudeMatrix[i][j];
		m_attitudeMatrix(i,j)=attitudeMatrix[i][j];
            }
        }

        Attitude::attToQuat();
        Attitude::quatToEuler();
        Attitude::quatToRodr();
}

Attitude::Attitude(const float rotationVector[3], const float rotationAngle){

        for (int i{0};i<=2;i++){
            m_rotationVector[i]=rotationVector[i];
        }
        m_rotationAngle=rotationAngle;

        Attitude::rodrToQuat();
        Attitude::quatToEuler();

}

void Attitude::quatToAtt(){

        float q1{m_q[0]};
        float q2{m_q[1]};
        float q3{m_q[2]};
        float q4{m_q[3]};

        /*m_attitudeMatrix[0][0]=q1*q1-q2*q2-q3*q3+q4*q4;
        m_attitudeMatrix[0][1]=2*(q1*q2+q3*q4);
        m_attitudeMatrix[0][2]=2*(q1*q3-q2*q4);
        m_attitudeMatrix[1][0]=2*(q2*q1-q3*q4);
        m_attitudeMatrix[1][1]=-q1*q1+q2*q2-q3*q3+q4*q4;
        m_attitudeMatrix[1][2]= 2*(q2*q3+q1*q4);
        m_attitudeMatrix[2][0]=2*(q3*q1+q2*q4);
        m_attitudeMatrix[2][1]=2*(q3*q2-q1*q4);
        m_attitudeMatrix[2][2]=-q1*q1-q2*q2+q3*q3+q4*q4;*/
	m_attitudeMatrix(0,0)=q1*q1-q2*q2-q3*q3+q4*q4;
        m_attitudeMatrix(0,1)=2*(q1*q2+q3*q4);
        m_attitudeMatrix(0,2)=2*(q1*q3-q2*q4);
        m_attitudeMatrix(1,0)=2*(q2*q1-q3*q4);
        m_attitudeMatrix(1,1)=-q1*q1+q2*q2-q3*q3+q4*q4;
        m_attitudeMatrix(1,2)= 2*(q2*q3+q1*q4);
        m_attitudeMatrix(2,0)=2*(q3*q1+q2*q4);
        m_attitudeMatrix(2,1)=2*(q3*q2-q1*q4);
        m_attitudeMatrix(2,2)=-q1*q1-q2*q2+q3*q3+q4*q4;

};

void Attitude::quatToEuler(){

        Attitude::quatToAtt();
	/*
        m_euler[1]=asin(m_attitudeMatrix[2][1]);                        // theta
        m_euler[0]=asin(m_attitudeMatrix[2][1]/cos(m_euler[1]));        // phi
        m_euler[2]=asin(m_attitudeMatrix[1][0]/cos(m_euler[1]));        // psi
	*/
	m_euler[1]=-asin(m_attitudeMatrix(2,0));                        // theta
        m_euler[0]=atan(m_attitudeMatrix(2,1)/m_attitudeMatrix(2,2));        // phi
        m_euler[2]=atan(m_attitudeMatrix(1,0)/m_attitudeMatrix(0,0));        // psi

};

void Attitude::quatToRodr(){

        m_rotationAngle=2*acos(m_q[3]);
        float tmp=sin(m_rotationAngle/2);
        m_rotationVector[0]=m_q[0]/tmp;
        m_rotationVector[1]=m_q[1]/tmp;
        m_rotationVector[2]=m_q[2]/tmp;

};

void Attitude::eulerToAtt(){

        float phi{m_euler[0]};
        float theta{m_euler[1]};
        float psi{m_euler[2]};
	/*
        m_attitudeMatrix[0][0]=cos(psi)*cos(theta);
        m_attitudeMatrix[1][0]=sin(psi)*cos(theta);
        m_attitudeMatrix[2][0]=-sin(theta);
        m_attitudeMatrix[0][1]=cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi);
        m_attitudeMatrix[1][1]=sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi);
        m_attitudeMatrix[2][1]=cos(theta)*sin(phi);
        m_attitudeMatrix[0][2]=cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
        m_attitudeMatrix[1][2]=sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
        m_attitudeMatrix[2][2]=cos(theta)*cos(phi);
	*/
	m_attitudeMatrix(0,0)=cos(psi)*cos(theta);
        m_attitudeMatrix(1,0)=sin(psi)*cos(theta);
        m_attitudeMatrix(2,0)=-sin(theta);
        m_attitudeMatrix(0,1)=cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi);
        m_attitudeMatrix(1,1)=sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi);
        m_attitudeMatrix(2,1)=cos(theta)*sin(phi);
        m_attitudeMatrix(0,2)=cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi);
        m_attitudeMatrix(1,2)=sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi);
        m_attitudeMatrix(2,1)=cos(theta)*cos(phi);
};

void Attitude::attToQuat(){
	
	/*
        float tmp[4]{m_attitudeMatrix[0][0],m_attitudeMatrix[1][1],m_attitudeMatrix[2][2],m_attitudeMatrix[0][0]+m_attitudeMatrix[1][1]+m_attitudeMatrix[2][2]};

        
        int maxElIdx{0};
        float maxEl{tmp[0]};
        for (int i{1};i<=3;i++){
            if (maxEl<tmp[i])
            {
                maxEl=tmp[i];
                maxElIdx=i;
            }
        }

        switch (maxElIdx){

        case 0:

            m_q[0]=1+m_attitudeMatrix[0][0]-m_attitudeMatrix[1][1]-m_attitudeMatrix[2][2];
            m_q[1]=m_attitudeMatrix[0][1]+m_attitudeMatrix[1][0];
            m_q[2]=m_attitudeMatrix[0][2]+m_attitudeMatrix[2][0];
            m_q[3]=m_attitudeMatrix[1][2]-m_attitudeMatrix[2][1];
            break;

        case 1:

            m_q[0]=m_attitudeMatrix[1][0]-m_attitudeMatrix[0][1];
            m_q[1]=1+m_attitudeMatrix[1][1]-m_attitudeMatrix[0][0]-m_attitudeMatrix[2][2];
            m_q[2]=m_attitudeMatrix[1][2]+m_attitudeMatrix[2][1];
            m_q[3]=m_attitudeMatrix[2][0]-m_attitudeMatrix[0][2];
            break;

        case 2:

            m_q[0]=m_attitudeMatrix[2][0]+m_attitudeMatrix[0][2];
            m_q[1]=m_attitudeMatrix[2][1]+m_attitudeMatrix[1][2];
            m_q[2]=1+m_attitudeMatrix[2][2]-m_attitudeMatrix[0][0]-m_attitudeMatrix[1][1];
            m_q[3]=m_attitudeMatrix[0][1]-m_attitudeMatrix[1][0];
            break;

        case 3:

            m_q[0]=m_attitudeMatrix[1][2]-m_attitudeMatrix[2][1];
            m_q[1]=m_attitudeMatrix[2][0]-m_attitudeMatrix[0][2];
            m_q[2]=m_attitudeMatrix[0][1]-m_attitudeMatrix[1][0];
            m_q[3]=1+m_attitudeMatrix[0][0]+m_attitudeMatrix[1][1]+m_attitudeMatrix[3][3];
            break;

        }
	*/

	        float tmp[4]{m_attitudeMatrix(0,0),m_attitudeMatrix(1,1),m_attitudeMatrix(2,2),m_attitudeMatrix.trace()};

        // find the column in which there is the maximum
        int maxElIdx{0};
        float maxEl{tmp[0]};
        for (int i{1};i<=3;i++){
            if (maxEl<tmp[i])
            {
                maxEl=tmp[i];
                maxElIdx=i;
            }
        }

        switch (maxElIdx){

        case 0:

            m_q[0]=1+m_attitudeMatrix(0,0)-m_attitudeMatrix(1,1)-m_attitudeMatrix(2,2);
            m_q[1]=m_attitudeMatrix(0,1)+m_attitudeMatrix(1,0);
            m_q[2]=m_attitudeMatrix(0,2)+m_attitudeMatrix(2,0);
            m_q[3]=m_attitudeMatrix(1,2)-m_attitudeMatrix(2,1);
            break;

        case 1:

            m_q[0]=m_attitudeMatrix(1,0)-m_attitudeMatrix(0,1);
            m_q[1]=1+m_attitudeMatrix(1,1)-m_attitudeMatrix(0,0)-m_attitudeMatrix(2,2);
            m_q[2]=m_attitudeMatrix(1,2)+m_attitudeMatrix(2,1);
            m_q[3]=m_attitudeMatrix(2,0)-m_attitudeMatrix(0,2);
            break;

        case 2:

            m_q[0]=m_attitudeMatrix(2,0)+m_attitudeMatrix(0,2);
            m_q[1]=m_attitudeMatrix(2,1)+m_attitudeMatrix(1,2);
            m_q[2]=1+m_attitudeMatrix(2,2)-m_attitudeMatrix(0,0)-m_attitudeMatrix(1,1);
            m_q[3]=m_attitudeMatrix(0,1)-m_attitudeMatrix(1,0);
            break;

        case 3:

            m_q[0]=m_attitudeMatrix(1,2)-m_attitudeMatrix(2,1);
            m_q[1]=m_attitudeMatrix(2,0)-m_attitudeMatrix(0,2);
            m_q[2]=m_attitudeMatrix(0,1)-m_attitudeMatrix(1,0);
            m_q[3]=1+m_attitudeMatrix(0,0)+m_attitudeMatrix(1,1)+m_attitudeMatrix(2,2);
            break;

        }


        // normalize the quaternion
        float norm{sqrt(m_q[0]*m_q[0]+m_q[1]*m_q[1]+m_q[2]*m_q[2]+m_q[3]*m_q[3])};

        for(int i{0};i<=3;i++)
        m_q[i]=m_q[i]/norm;

};

void Attitude::rodrToQuat(){

        m_q[0]=m_rotationVector[0]*sin(m_rotationAngle/2);
        m_q[1]=m_rotationVector[1]*sin(m_rotationAngle/2);
        m_q[2]=m_rotationVector[2]*sin(m_rotationAngle/2);
        m_q[3]=cos(m_rotationAngle/2);

};


void Attitude::getQuaternion(float quaternion[4]){
	for(int i{1};i<=3;i++)	
	quaternion[i]=m_q[i];
};

void Attitude::getEuler(float euler[3]){
	for(int i{1};i<=2;i++)	
	euler[i]=m_euler[i];
};

void Attitude::getAttitudeMatrix(float attitudeMatrix[3][3]){
	for (int i{0};i<=2;i++){
		for(int j{0};j<=2;j++){
			//attitudeMatrix[i][j]=m_attitudeMatrix[i][j];
			attitudeMatrix[i][j]=m_attitudeMatrix(i,j);		
		}	
	}
};

void Attitude::getAttitudeMatrix(Eigen::Matrix3f attitudeMatrix){
	for (int i{0};i<=2;i++){
		for(int j{0};j<=2;j++){
			//attitudeMatrix[i][j]=m_attitudeMatrix[i][j];
			attitudeMatrix(i,j)=m_attitudeMatrix(i,j);		
		}	
	}
};

void Attitude::getRotationParams(float rotationVector[3],float &rotationAngle){
	for(int i{1};i<=2;i++)	
	rotationVector[i]=m_rotationVector[i];
	rotationAngle=m_rotationAngle;
};

void Attitude::updateAttitude(const float q1, const float q2, const float q3, const float q4){
        m_q[0]=q1;
        m_q[1]=q2;
        m_q[2]=q3;
        m_q[3]=q4;

        Attitude::quatToEuler();    // it calls quatToAtt!
        Attitude::quatToRodr();
};

void Attitude::updateAttitude(const float phi,const float theta,const float psi){

        m_euler[0]=phi;
        m_euler[1]=theta;
        m_euler[2]=psi;

        Attitude::eulerToAtt();
        Attitude::attToQuat();
        Attitude::quatToRodr();
};

void Attitude::updateAttitude(const float attitudeMatrix[3][3]){

        for (int i{0};i<=2;i++){
            for (int j{0};j<=2;j++){
                //m_attitudeMatrix[i][j]=attitudeMatrix[i][j];
                m_attitudeMatrix(i,j)=attitudeMatrix[i][j];
            }
        }

        Attitude::attToQuat();
        Attitude::quatToEuler();
        Attitude::quatToRodr();
};

void Attitude::updateAttitude(const float rotationVector[3], const float rotationAngle){

        for (int i{0};i<=2;i++){
            m_rotationVector[i]=rotationVector[i];
        }
        m_rotationAngle=rotationAngle;

        Attitude::rodrToQuat();
        Attitude::quatToEuler();


};

/*

void Attitude::displayAttitude(){

	std::cout<<"\n Printing quaternion"<<std::endl;
	for (int i{0};i<=3;i++){
		std::cout<<"q_"<< i << "="<<m_q[i]<<std::endl;	
	}	
	
	std::cout<<"\n Printing Euler"<<std::endl;
	std::cout<<"phi="<<m_euler[0]<<std::endl;	
	std::cout<<"theta="<<m_euler[1]<<std::endl;
	std::cout<<"psi="<<m_euler[2]<<std::endl;

	std::cout<<"\n Printing attitude matrix"<<std::endl;
	for(int i{0};i<=2;i++){
		for(int j{0};j<=2;j++){
			//std::cout<<m_attitudeMatrix[i][j]<<'\t';		
			std::cout<<m_attitudeMatrix(i,j)<<'\t';		
		}	
		std::cout<<std::endl;
	}

	std::cout<<"\n Printing rotation vector"<<std::endl;
	for(int i{0};i<=2;i++){
			std::cout<<m_rotationVector[i]<<'\n';		
	}	
	std::cout<<"\n Printing rotation angle"<<std::endl;
			std::cout<<m_rotationAngle<<'\n';		
	
}

*/
