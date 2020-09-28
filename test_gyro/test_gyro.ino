
#include <stlport.h>
#include "MyMPU6050.h"
#include <Wire.h>
#include "AttitudeFilter.h"

// define the object to handle IMU

MyMPU6050 mpu6050(Wire);
long timer = 0;


void setup() {
Serial.begin(9600);
Wire.begin();
mpu6050.begin();
mpu6050.calcGyroOffsets(true);
mpu6050.computeAccOffset(true);
Serial.print('\n');
}

// define the object for the attitude estimator
float updateTime{150}; // milliseconds
float sigmaAcc{mpu6050.getAccelerometerNoiseSigma()*100};
float sigmaRRW{sigmaAcc};
float sigmaARW{sigmaAcc};
MEKF attitudeEstimator(updateTime*1e-3, sigmaRRW, sigmaARW, sigmaAcc);
float euler[3];

void loop() {


  if(millis() - timer > updateTime){

    // obtain the measurement
    mpu6050.update();

 /*   // compute the kalman gain
    attitudeEstimator.filterGainComputation();
    
    // correction
    attitudeEstimator.filterCorrectAccelerometer(mpu6050.getAccX(), mpu6050.getAccY(), mpu6050.getAccZ());
    attitudeEstimator.getEulerAngles(euler);
    Serial.print(euler[0]*180/PI);
    Serial.print('\t');
    Serial.print(euler[1]*180/PI);
    Serial.print('\t');
    Serial.println(euler[2]*180/PI);*/
    // prediction
    attitudeEstimator.filterPredict(mpu6050.getGyroX()*PI/180, mpu6050.getGyroY()*PI/180, mpu6050.getGyroZ()*PI/180);
    attitudeEstimator.getEulerAngles(euler);

    timer = millis();
    Serial.print(euler[0]*180/PI);
    Serial.print('\t');
    Serial.print(euler[1]*180/PI);
    Serial.print('\t');
    Serial.println(euler[2]*180/PI);

 //   Serial.println("Ciao");
  }

}
