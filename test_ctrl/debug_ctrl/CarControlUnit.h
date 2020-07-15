/*
Definition of an object to manage the control of the car given the setpoints commanded
*/

#ifndef CARCONTROLUNIT_H_INCLUDED
#define CARCONTROLUNIT_H_INCLUDED

#include "Freenove_WS2812B_RGBLED_Controller.h"
#include "Servo.h"

// Motor definitions
#define MAX_PWM 255
#define MIN_PWM 0
#define MOTOR_PWM_DEAD 5

#define PIN_DIRECTION_LEFT 4
#define PIN_DIRECTION_RIGHT 3
#define PIN_MOTOR_PWM_LEFT 6
#define PIN_MOTOR_PWM_RIGHT 5

// Servo definitions
#define PIN_SERVO 2

// Tracking sensor definition
#define PIN_TRACKING_LEFT A1
#define PIN_TRACKING_CENTER A2
#define PIN_TRACKING_RIGHT A3


// Battery and Buzzer definitions
#define PIN_BATTERY A0
#define PIN_BUZZER A0

#define BATTERY_VOL_STANDARD 7.0


// LED definitions
#define I2C_ADDRESS  0x20
#define LEDS_COUNT 10

// Motor characteristic curve (obtained from experiments, see related folder in the project)
#define MOTOR_CURVE_SLOPE 361.77
#define MOTOR_CURVE_INTERCEPT -105.96

// Car kinematic parameter (obtained from experiments, see the related folder in the project)
#define EQUIVALENT_Y0 0.1387

class CarControlUnit{

private:

    float m_batteryVoltage{0};												// voltage of the battery
    bool m_isBuzzered{false};												// flag used to switch from battery read to sound emitter (attached to the same PIN)
    Freenove_WS2812B_Controller m_LEDController=Freenove_WS2812B_Controller(I2C_ADDRESS, LEDS_COUNT, TYPE_GRB);		// object used to control the LEDs
    Servo m_servo;													// object used to control the car servo
    u8 m_trackingSensorValue[4];											// outputs of the tracking sensor (front car)
    // The following 4 parameters are modified once the setpoint tracking routine is called 
    double m_wheelSpeedLeft;												// commanded speed to the left wheel
    double m_wheelSpeedRight;												// commanded speed to the right wheel 
    int m_PWMl; 													// commanded PWM to the left block of motors      
    int m_PWMr;														// commanded PWM to the right block of motors
    double m_equivalentSemiBase{EQUIVALENT_Y0};  									// kinematic parameter relating wheel speed to the angular velocity

    bool m_motorPins{false}; 												// flag to see if the motor pins have been defined 
    bool m_servoPins{false};												// flag to see if the servo pins have been defined
    bool m_trackingSensorPins{false};											// flag to see if the tracking sensor pins have been defined


public:

    // getters

    float getBatteryVoltage();
    /* 
    return the level of the battery voltage
    */

    bool getIsBuzzered();
    /* 
    return the state of the buz flag
    */

    double getSpeedLeft();
    /* 
    return the value of the left wheel speed
    */

    double getSpeedRight();
    /* 
    return the level of the right wheel speed
    */

    int getPWMLeft();
    /* 
    return the PWM level of the left motors
    */

    int getPWMRight();
    /* 
    return the PWM level of the right motors
    */
  
    // setters

    void setBatteryVoltage();
    /* 
    read and set the level of the battery
    */

    void setIsBuzzered(bool isBuzzered);
    /* 
    set the level of the buzzer flag to isBuzzered
    */

    void setupMotorPins();
    /* 
    setup the pins of the motors and set the corresponding flag
    */

    void setupServoPins();
    /* 
    setup the pins of the servo and set the corresponding flag
    */

    void setupTrackingSensorPins();
    /* 
    setup the pins of the tracking sensor and set the corresponding flag
    */

    void commandMotorPWM(int pwmL,int pwmR);
    /* 
    low level commands to the motors
    */

    void setBuzzer (bool flag);
    /* 
    setthe buzzer (true= emit sound)
    */

    void turnOnCarLights(u8 red, u8 green, u8 blue);
    /* 
    switch on the car lights to the assigned RGB sequence
    */

    void resetCarAction ();
    /* 
    stop any car motion
    */

    bool readTrackingSensor();
    /* 
    read measurements from the tracking sensor and write them in m_trackingSensorValue; return true if readings are available
    */   

    void setServoAngle(int servoAngle);
    /* 
    set the angle of the servo
    */   

    int readServoAngle();
    /* 
    read the current angle of the servo
    */


    // convert the speed of the wheel in PWM
    int fromSpeedToPWM(double wheelSpeed);
    /* 
    convert the speed of the wheel in PWM using experimental characterization (inverse feedforward control)
    */

    // solve the kinematic problem wheel speeds form kinematic (set the members of this class)
    void inverseKinematics(double linearSpeed, double angularSpeed);
    /* 
    solve the kineamtics problem, i.e. map the value of the car speed and angular yaw rate in speed of the single wheels and set the corresponding ,embers of the object
    */

    void trackSpeedSetpoints(double linearSpeed, double angularSpeed);
    /* 
    move the car (i.e. command the motor PWMs) to track the desired values of linear and angular speed
    */

};


#endif // CARCONTROLUNIT_H_INCLUDED
