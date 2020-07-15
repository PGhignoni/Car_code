#include "CarControlUnit.h"
#include "RadioReceiver.h"
#include "CarPlanner.h"


CarControlUnit CCU;						// object to control the car
RadioReceiver radioUnit;					// object to decode the data from the remote 
int *data;							// pointer for the data received from the remote
CarPlanner carPlanner;						// object for car planning 

void setup() {
  

  CCU.setupMotorPins();
  CCU.setupServoPins();

  delay(1000);
  CCU.setServoAngle(90);

  radioUnit.setupReceiver();
  CCU.turnOnCarLights(255,0,0);					// turn on the lights
  
/*  radioUnit.setupReceiver();
  if (!radioUnit.isListening()){
    CCU.setBuzzer(true);
    delay(1000);
  }
*/  
}

void loop() {
  // put your main code here, to run repeatedly:

  // Test motor low level commands 
/*
  CCU.commandMotorPWM(100,-100);
  CCU.setServoAngle(45);
  delay(1000);

  CCU.commandMotorPWM(-100,100);
  CCU.setServoAngle(90+45);
  delay(1000);

  CCU.resetCarAction();
  delay(1000);
*/
  // Test Radio Coms
  radioUnit.receiveData();

  if (radioUnit.isReceiving())
  {
  data=radioUnit.getData();
  
  /*Serial.println(*(data+5));
  //radioUnit.printData();
  Serial.print(data[3]);
  Serial.print('\n');
*/
  carPlanner.setCarMode(data);
  carPlanner.setSetpoints(data);

  Serial.print("Mode||V||Omega||Curvature||SpeedL||SpeedR||PWMl||PWMr||P1||P2 \t");
  Serial.print(carPlanner.getMode());
  Serial.print('\t');
  Serial.print(carPlanner.getLinearVelocity());
  Serial.print('\t');
  Serial.print(carPlanner.getAngularVelocity());
  Serial.print('\t');
  Serial.print(carPlanner.getCurvature());
  Serial.print('\t'); 
  CCU.trackSpeedSetpoints(carPlanner.getLinearVelocity(),carPlanner.getAngularVelocity());
  Serial.print(CCU.getSpeedLeft());
  Serial.print('\t');
  Serial.print(CCU.getSpeedRight());
  Serial.print('\t');
  Serial.print(CCU.getPWMLeft());
  Serial.print('\t');
  Serial.print(CCU.getPWMRight());
  Serial.print('\t');
  Serial.print(*data);
  Serial.print('\t');
  Serial.print(*(data+1));
  Serial.print('\n');
  }
  else
  {
    CCU.resetCarAction();								// stay still if the communication is not available
  }
}
