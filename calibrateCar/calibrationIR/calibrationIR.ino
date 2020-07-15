/**********************************************************************
  Filename    : IR_Remote_Car.ino
  Product     : Freenove 4WD Car for UNO
  Description : An IR-Remote Car.
  Auther      : www.freenove.com
  Modification: 2019/08/06
**********************************************************************/
#include <IRremote.h>
#include "Freenove_4WD_Car_for_Arduino.h"

//define key, the code can not be changed.
#define IR_REMOTE_KEYCODE_UP      0xFF02FD
#define IR_REMOTE_KEYCODE_DOWN    0xFF9867
#define IR_REMOTE_KEYCODE_LEFT    0xFFE01F
#define IR_REMOTE_KEYCODE_RIGHT   0xFF906F
#define IR_REMOTE_KEYCODE_CENTER  0xFFA857

#define IR_UPDATE_TIMEOUT     120
#define IR_CAR_SPEED          240                 // speeds to be used for the calibration 60 120 180 240  (3x measurements)

IRrecv irrecv(PIN_IRREMOTE_RECV);
decode_results results;
u32 currentKeyCode, lastKeyCode;
bool isStopFromIR = false;
u32 lastIRUpdateTime = 0;
float batteryVoltage = 0;
//bool isBuzzered = false;

void setup() {
  irrecv.enableIRIn(); // Start the receiver
}

void loop() {
  if (irrecv.decode(&results)) {
    isStopFromIR = false;
    currentKeyCode = results.value;
    if (currentKeyCode != 0xFFFFFFFF) {
      lastKeyCode = currentKeyCode;
    }
    switch (lastKeyCode) {
      case IR_REMOTE_KEYCODE_UP:
        motorRun(IR_CAR_SPEED, IR_CAR_SPEED); //move forward
        break;
      case IR_REMOTE_KEYCODE_DOWN:
        motorRun(-IR_CAR_SPEED, -IR_CAR_SPEED); //move back
        break;
      case IR_REMOTE_KEYCODE_LEFT:
        motorRun(-IR_CAR_SPEED, IR_CAR_SPEED);  //turn left
        break;
      case IR_REMOTE_KEYCODE_RIGHT:
        motorRun(IR_CAR_SPEED, -IR_CAR_SPEED);  //turn right
        break;
      case IR_REMOTE_KEYCODE_CENTER:
        setBuzzer(true);              //turn on buzzer
        break;
      default:
        break;
    }
    irrecv.resume(); // Receive the next value
    lastIRUpdateTime = millis(); //write down current time
  }
  else {
    if (millis() - lastIRUpdateTime > IR_UPDATE_TIMEOUT) {
      if (!isStopFromIR) {
        isStopFromIR = true;
        motorRun(0, 0);
        setBuzzer(false);
      }
      lastIRUpdateTime = millis();
    }
  }
}


/*

  void pinsSetup() {
  pinMode(PIN_DIRECTION_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_LEFT, OUTPUT);
  pinMode(PIN_DIRECTION_RIGHT, OUTPUT);
  pinMode(PIN_MOTOR_PWM_RIGHT, OUTPUT);

  pinMode(PIN_SONIC_TRIG, OUTPUT);// set trigPin to output mode
  pinMode(PIN_SONIC_ECHO, INPUT); // set echoPin to input mode

  pinMode(PIN_TRACKING_LEFT, INPUT); //
  pinMode(PIN_TRACKING_RIGHT, INPUT); //
  pinMode(PIN_TRACKING_CENTER, INPUT); //

  setBuzzer(false);
  }

  void motorRun(int speedl, int speedr) {
  int dirL = 0, dirR = 0;
  if (speedl > 0) {
    dirL = 0;
  }
  else {
    dirL = 1;
    speedl = -speedl;
  }

  if (speedr > 0) {
    dirR = 1;
  }
  else {
    dirR = 0;
    speedr = -speedr;
  }

  speedl = constrain(speedl, 0, 255);
  speedr = constrain(speedr, 0, 255);

  if (abs(speedl) < MOTOR_PWM_DEAD && abs(speedr) < MOTOR_PWM_DEAD) {
    speedl = 0;
    speedr = 0;
  }

  digitalWrite(PIN_DIRECTION_LEFT, dirL);
  digitalWrite(PIN_DIRECTION_RIGHT, dirR);
  analogWrite(PIN_MOTOR_PWM_LEFT, speedl);
  analogWrite(PIN_MOTOR_PWM_RIGHT, speedr);
  }


  bool getBatteryVoltage() {
  if (!isBuzzered) {
    pinMode(PIN_BATTERY, INPUT);
    int batteryADC = analogRead(PIN_BATTERY);
    if (batteryADC < 614)   // 3V/12V ,Voltage read: <2.1V/8.4V
    {
      batteryVoltage = batteryADC / 1023.0 * 5.0 * 4;
      return true;
    }
  }
  return false;
  }
  void setBuzzer(bool flag) {
  isBuzzered = flag;
  pinMode(PIN_BUZZER, flag);
  digitalWrite(PIN_BUZZER, flag);
  }
  void alarm(u8 beat, u8 repeat) {
  beat = constrain(beat, 1, 9);
  repeat = constrain(repeat, 1, 255);
  for (int j = 0; j < repeat; j++) {
    for (int i = 0; i < beat; i++) {
      setBuzzer(true);
      delay(100);
      setBuzzer(false);
      delay(100);
    }
    delay(500);
  }
  }
  void resetCarAction() {
  motorRun(0, 0);
  setBuzzer(false);
  }

*/
