#include "Freenove_WS2812B_RGBLED_Controller.h"
//#include "Arduino.h"
#include "Servo.h"
#include "CarControlUnit.h"


float CarControlUnit::getBatteryVoltage()
{
    return m_batteryVoltage;
}


bool CarControlUnit::getIsBuzzered()
{
    return m_isBuzzered;
}



double CarControlUnit::getSpeedLeft(){
    return m_wheelSpeedLeft;  
}



double CarControlUnit::getSpeedRight(){
    return m_wheelSpeedRight;  
}



int CarControlUnit::getPWMLeft()
{
    return m_PWMl;  
}



int CarControlUnit::getPWMRight()
{
    return m_PWMr;  
}



void CarControlUnit::setBatteryVoltage()
{
    if (!m_isBuzzered)
    {
		pinMode(PIN_BATTERY, INPUT);
		int batteryADC = analogRead(PIN_BATTERY);
		if (batteryADC < 614)		// 3V/12V ,Voltage read: <2.1V/8.4V
		{
			m_batteryVoltage = batteryADC / 1023.0 * 5.0 * 4;
		}
	}
}



void CarControlUnit::setIsBuzzered(bool isBuzzered)
{
    m_isBuzzered= isBuzzered;
}



void CarControlUnit::setupMotorPins()
{
    pinMode(PIN_DIRECTION_LEFT,OUTPUT);
    pinMode(PIN_DIRECTION_RIGHT,OUTPUT);
    pinMode(PIN_MOTOR_PWM_LEFT,OUTPUT),
    pinMode(PIN_MOTOR_PWM_RIGHT,OUTPUT);
    m_motorPins=true;
}



void CarControlUnit::setupServoPins()
{
     m_servo.attach(PIN_SERVO);
     m_servo.write(90);
     m_servoPins=true;
}



void CarControlUnit::setupTrackingSensorPins()
{
    pinMode(PIN_TRACKING_LEFT,INPUT);
    pinMode(PIN_TRACKING_CENTER,INPUT);
    pinMode(PIN_TRACKING_RIGHT,INPUT);
    m_trackingSensorPins=true;
}



void CarControlUnit::commandMotorPWM(int PWMl, int PWMr)
{
    // avoid checking the initialization of m_pinsMotor
    int dirL = 0, dirR = 0;

    // setup the correct direction of the motion
    if (PWMl > 0)
    {
	dirL = 0;
    }
    else
    {
	dirL = 1;
	PWMl = -PWMl;
    }

    if (PWMr > 0)
    {
	dirR = 1;
    }
    else
    {
	dirR = 0;
	PWMr = -PWMr;
    }

    // ensure correct PWM range
    PWMl = constrain(PWMl, MIN_PWM, MAX_PWM);
    PWMr = constrain(PWMr, MIN_PWM, MAX_PWM);

    // avoid too low PWM
    if (abs(PWMl) < MOTOR_PWM_DEAD && abs(PWMr) < MOTOR_PWM_DEAD)
    {
	PWMl = 0;
	PWMr = 0;
    }

    digitalWrite(PIN_DIRECTION_LEFT, dirL);
    digitalWrite(PIN_DIRECTION_RIGHT, dirR);
    analogWrite(PIN_MOTOR_PWM_LEFT, PWMl);
    analogWrite(PIN_MOTOR_PWM_RIGHT, PWMr);
}



void CarControlUnit::setBuzzer(bool flag)
{
    setIsBuzzered(flag);
    pinMode(PIN_BUZZER,flag);
    digitalWrite(PIN_BUZZER, flag);
}



void CarControlUnit::turnOnCarLights(u8 red, u8 green, u8 blue)
{
    while (!m_LEDController.begin()); 					// wait until the LED controller is working
    m_LEDController.setAllLedsColor(red, green, blue);
}



void CarControlUnit::resetCarAction()
{
    commandMotorPWM(0,0);
    setBuzzer(false);
    if (m_servoPins)
    {
        m_servo.write(90);
    }
}



bool CarControlUnit::readTrackingSensor()
{
    if(m_trackingSensorPins)
    {
        m_trackingSensorValue[0] = digitalRead(PIN_TRACKING_LEFT);
        m_trackingSensorValue[1] = digitalRead(PIN_TRACKING_CENTER);
        m_trackingSensorValue[2] = digitalRead(PIN_TRACKING_RIGHT);
        m_trackingSensorValue[3] = m_trackingSensorValue[0] << 2 | m_trackingSensorValue[1] << 1 | m_trackingSensorValue[2];
        return true;
    }
    return false;
}



void CarControlUnit::setServoAngle(int servoAngle)
{
    if(m_servoPins)
    {
        servoAngle=constrain(servoAngle,0,180);
        m_servo.write(servoAngle);
    }
}



int CarControlUnit::readServoAngle()
{
    if(m_servoPins)
    {   int outVar;
        outVar=m_servo.read();
        return outVar;
    }
    return -1;
}



int CarControlUnit::fromSpeedToPWM(double wheelSpeed)
{
    if (abs(wheelSpeed)<0.6)
    {
    // under the threshold value found during the experiments use a linear relationship mapping the 0PWM to zero speed
    double tmp=(0.6*MOTOR_CURVE_SLOPE+MOTOR_CURVE_INTERCEPT)*wheelSpeed/0.6;
    return (int)tmp;
    }
    else{
    double tmp=abs(wheelSpeed)*MOTOR_CURVE_SLOPE+MOTOR_CURVE_INTERCEPT;
    if (wheelSpeed<0)
    tmp=-tmp;
    return (int)tmp;
    }
}



void CarControlUnit::inverseKinematics(double linearSpeed, double angularSpeed)
{
    m_wheelSpeedLeft=linearSpeed-m_equivalentSemiBase*angularSpeed;
    m_wheelSpeedRight=linearSpeed+m_equivalentSemiBase*angularSpeed;
}



void CarControlUnit::trackSpeedSetpoints(double linearSpeed, double angularSpeed)
{
     this->inverseKinematics(linearSpeed,angularSpeed);
     int PWMl{this->fromSpeedToPWM(this->m_wheelSpeedLeft)};
     int PWMr{this->fromSpeedToPWM(this->m_wheelSpeedRight)};
     m_PWMl=PWMl;
     m_PWMr=PWMr;
     this->commandMotorPWM(PWMl,PWMr);     
}
