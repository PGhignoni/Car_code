#ifndef CARCONTROLUNIT_H_INCLUDED
#define CARCONTROLUNIT_H_INCLUDED

#include "Freenove_WS2812B_RGBLED_Controller.h"
//#include "Arduino.h"
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

// Motor characteristic curve
#define MOTOR_CURVE_SLOPE 361.77
#define MOTOR_CURVE_INTERCEPT -105.96

// Car kinematic parameter
#define EQUIVALENT_Y0 0.1387

class CarControlUnit{

private:

    float m_batteryVoltage{0};
    bool m_isBuzzered{false};
    Freenove_WS2812B_Controller m_LEDController=Freenove_WS2812B_Controller(I2C_ADDRESS, LEDS_COUNT, TYPE_GRB);
    Servo m_servo;
    u8 m_trackingSensorValue[4];
    // The following 4 parameters are modified once the setpoint tracking routine is called 
    double m_wheelSpeedLeft;
    double m_wheelSpeedRight;
    int m_PWMl;       
    int m_PWMr;
    double m_equivalentSemiBase{EQUIVALENT_Y0};  // kinematic parameter

    bool m_motorPins{false}; // true if the corresponding pins have been set
    bool m_servoPins{false};
    bool m_trackingSensorPins{false};


public:

    // getters
    float getBatteryVoltage();

    bool getIsBuzzered();

    double getSpeedLeft();

    double getSpeedRight();

    int getPWMLeft();

    int getPWMRight();
  
    // setters
    void setBatteryVoltage();

    void setIsBuzzered(bool isBuzzered);

    // setup the pins of the motor controller
    void setupMotorPins();

    // setup the pins for the servo
    void setupServoPins();

    // setup the pins for the tracking sensor
    void setupTrackingSensorPins();

    // low level command to the motors
    void commandMotorPWM(int pwmL,int pwmR);

    // buzzer
    void setBuzzer (bool flag);

    // switch on the lights of the car
    void turnOnCarLights(u8 red, u8 green, u8 blue);

    // stop any car motion
    void resetCarAction ();

    // read the measurements from the tracking sensor
    bool readTrackingSensor();

    // set the servo angle
    void setServoAngle(int servoAngle);

    // read the angle of the servo
    int readServoAngle();

    // convert the speed of the wheel in PWM
    int fromSpeedToPWM(double wheelSpeed);

    // solve the kinematic problem wheel speeds form kinematic (set the members of this class)
    void inverseKinematics(double linearSpeed, double angularSpeed);

    // track the setpoints of speed and rotational velocity
    void trackSpeedSetpoints(double linearSpeed, double angularSpeed);

};


#endif // CARCONTROLUNIT_H_INCLUDED
