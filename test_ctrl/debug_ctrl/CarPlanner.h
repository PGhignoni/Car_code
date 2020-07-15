/*
Class implementing the planning strategy, i.e. find the speed and angular speed setpoints given the commands by the user
*/


#ifndef CARPLANNER_H_INCLUDED
#define CARPLANNER_H_INCLUDED

#include <stdlib.h>



// stick from the radio commands

#define NEUTRAL_STICK_POSITION 512
#define MIN_STICK_VALUE 0
#define MAX_STICK_VALUE 1023
#define STICK_TOLERANCE 50



// kinematic parameters for the control of the car

#define MAX_LINEAR_SPEED 0.9
#define MAX_ANGULAR_SPEED 6.5



// radius of the car during the automatic turns

#define MAX_RADIUS 15
#define MIN_RADIUS 0.5



// Define the control modes

enum class ControlMode
{
    MODE_STOP,                  // The car is still
    MODE_MANUAL,                // The car moves according to manual commands
    MODE_AUTO                   // The car moves autonomously
};






class CarPlanner{

    private:

    ControlMode m_carControlMode;					// control mode of the car
    double m_linearVelocitySp;						// linear velocity computed setpoint
    double m_angularVelocitySp;						// angular rate computed setpoint
    double m_curvatureSp; 						// equivalent radius of turn: V=R*\omega


    public:

    CarPlanner(): m_carControlMode{ControlMode::MODE_STOP}, m_linearVelocitySp{0.0}, m_angularVelocitySp{0.0}, m_curvatureSp{0.0}{};
    /*
    default constructor
    */

    // getters

    int getMode();
    /*
    return the current car mode (e.g., 0 || 1 || 2)
    */
    

    double getLinearVelocity();
    /*
    return the current linear speed setpoint
    */


    double getAngularVelocity();
    /*
    return the current angular rate setpoint 
    */


    double getCurvature();
    /*
    return the current value of the radius of turn
    */

  
    void setCarMode(int* dataController);
    /*
    set the current car mode according to the data received from the radio
    */


    void setSetpoints(int* dataController);
    /*
    real and simple planner: set the speed and angular rate setpoints according to the data received from the radio
    */

};




#endif // CARPLANNER_H_INCLUDED
