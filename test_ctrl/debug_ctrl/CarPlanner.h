#ifndef CARPLANNER_H_INCLUDED
#define CARPLANNER_H_INCLUDED

#include <stdlib.h>


#define NEUTRAL_STICK_POSITION 512
#define MIN_STICK_VALUE 0
#define MAX_STICK_VALUE 1023
#define STICK_TOLERANCE 50

#define MAX_LINEAR_SPEED 0.9
#define MAX_ANGULAR_SPEED 6.5

#define MAX_RADIUS 15
#define MIN_RADIUS 0.5

// Defines the control modes
enum class ControlMode
{
    MODE_STOP,                  // The car is still
    MODE_MANUAL,                // The car moves according to manual commands
    MODE_AUTO                   // The car moves autonomously
};


class CarPlanner{

    private:

    ControlMode m_carControlMode;
    double m_linearVelocitySp;
    double m_angularVelocitySp;
    double m_curvatureSp; // V=R*\omega


    public:

    CarPlanner(): m_carControlMode{ControlMode::MODE_STOP}, m_linearVelocitySp{0.0}, m_angularVelocitySp{0.0}, m_curvatureSp{0.0}{};

    // getters
    int getMode();

    double getLinearVelocity();

    double getAngularVelocity();

    double getCurvature();

    // Set the control mode as a function of the information received by the radio
    void setCarMode(int* dataController);

    // set the speed setpoints as a function of the information received by the radio
    void setSetpoints(int* dataController);

};




#endif // CARPLANNER_H_INCLUDED
