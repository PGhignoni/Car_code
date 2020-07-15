#include <stdlib.h>
#include "CarPlanner.h"


int CarPlanner::getMode()
{
    return (int)m_carControlMode;
}



double CarPlanner::getLinearVelocity()
{
    return m_linearVelocitySp;
}



double CarPlanner::getAngularVelocity()
{
    return m_angularVelocitySp;
}



double CarPlanner::getCurvature()
{
    return m_curvatureSp;
}



void CarPlanner::setCarMode(int* data)
{
    switch (*(data+5))
    {
    case 0:
        m_carControlMode=ControlMode::MODE_STOP;
        break;

    default:
        switch (*(data+6))
        {

        case 1:
            m_carControlMode=ControlMode::MODE_MANUAL;
            break;

        default:
            if (*(data+7)==1)
            {
                m_carControlMode=ControlMode::MODE_AUTO;
            }
            else
            {
                m_carControlMode=ControlMode::MODE_STOP;
            }

        }
    }
}




void CarPlanner::setSetpoints(int* data)
{
        // perform different actions depending on the control mode

        switch (m_carControlMode)
        {
            case ControlMode::MODE_STOP:

                // stay still
                m_linearVelocitySp=0.0;
                m_angularVelocitySp=0.0;
                m_curvatureSp=0.0;
            break;

            case ControlMode::MODE_MANUAL:
            {
                int x= *(data+2);  		// command for the linear speed
                int y= *(data+3);  		// command for the angular speed
                int P1= *data; 			// the potentiometer sets the speed value

                // effective commands reducing biases
                int netX= x-NEUTRAL_STICK_POSITION;
                int netY= y-NEUTRAL_STICK_POSITION;

                // introduce a deadzone to compensate steady state biases in x and y
                if(abs(netX)<STICK_TOLERANCE)
                {
                     netX=0;
                }
                if(abs(netY)<STICK_TOLERANCE)
                {
                     netY=0;
                }
                // setpoints maximum absolute values are proportional to the potentiometer
                m_linearVelocitySp=(double)netX*2/MAX_STICK_VALUE*P1/MAX_STICK_VALUE*MAX_LINEAR_SPEED;
                m_angularVelocitySp=(double)netY*2/MAX_STICK_VALUE*P1/MAX_STICK_VALUE*MAX_ANGULAR_SPEED;
                if (m_angularVelocitySp<0.01)
                {
                    m_curvatureSp=10000.0;	// avoid numerical issues for too small angula rates 
                }
                else
                {
                    m_curvatureSp=m_linearVelocitySp/m_angularVelocitySp;
                }

            break;
            }

            case ControlMode::MODE_AUTO:
            {

            int x= *(data+2);           // commands the motion if its value is outside the deadzone (positive turn left, negative turn right)
            int P1= *data;              // the potentiometer sets the speed value
            int P2= *(data+1);          // the potentiometer sets the radious of curvature

            // effective commands reducing biases
            int netX= x-NEUTRAL_STICK_POSITION;
            if(abs(netX)<STICK_TOLERANCE)
                {
                     netX=0;                    // in this case the car does not move
                }

            m_linearVelocitySp=(double)abs(netX)*2/MAX_STICK_VALUE*P1/MAX_STICK_VALUE*MAX_LINEAR_SPEED;		// linear speed proportional to potentiometer value
            m_curvatureSp=MIN_RADIUS + (double)P2/MAX_STICK_VALUE*(MAX_RADIUS-MIN_RADIUS);			// radius of turn proportional to P2
	    if (netX<=0){            
            m_angularVelocitySp=-m_linearVelocitySp/m_curvatureSp; // turn right
	    }
	    else{
            m_angularVelocitySp=m_linearVelocitySp/m_curvatureSp; // turn left
            }            
            break;
            }


            default:
                m_linearVelocitySp=0.0;
                m_angularVelocitySp=0.0;
                m_curvatureSp=0.0;

        }


}
