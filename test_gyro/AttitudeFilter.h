#ifndef ATTITUDEFILTER_H_INCLUDED
#define ATTITUDEFILTER_H_INCLUDED

#include "Attitude.h"

class AttitudeFilter{

    protected:

        //filter time step
        float m_dt;

        //Attitude representation
	Attitude m_attitude;
	
        // angular speed
        float m_omega[3];


        AttitudeFilter(float dt): m_dt{dt}, m_attitude{0.0,0.0,0.0,1.0}, m_omega{0.0,0.0,0.0}
        {
        }


};


class MEKF: public AttitudeFilter{

        public:

            MEKF(float dt): AttitudeFilter{dt}
            {
            }

	    void displayFilterStates();

};



#endif // ATTITUDEFILTER_H_INCLUDED
