//#include <MPU6050_tockn.h>
#include "MyMPU6050.h"
#include <Wire.h>
#include "Estimator.h"
#include "Filter.h"
#include "AttitudeFilter.h"

// define the object to handle IMU
//MPU6050 mpu6050(Wire);
MyMPU6050 mpu6050(Wire);
long timer = 0;

// define the object to handle estimator with the selected update rate
float freqUpdate{10}; // update frequency of controlUnit
float updateTime{1/freqUpdate};
Estimator estimator{updateTime};


// create the filter for the accelerometer.
float freqCutoff{5}; // cutoff frequency of the filter [Hz]
float cutoff{1/freqCutoff};

Filter filter_ax{updateTime,cutoff};
Filter filter_ay{updateTime,cutoff};
Filter filter_az{updateTime,cutoff};

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.computeAccOffset(true);
}

void loop() {
  mpu6050.update();

  if(millis() - timer > updateTime*1000){

//   Serial.println("=======================================================");
/*    Serial.print("temp : ");Serial.println(mpu6050.getTemp());
    Serial.print("accX : ");Serial.print(mpu6050.getAccX());
    Serial.print("\taccY : ");Serial.print(mpu6050.getAccY());
    Serial.print("\taccZ : ");Serial.println(mpu6050.getAccZ());

    Serial.print("gyroX : ");Serial.print(mpu6050.getGyroX());
    Serial.print("\tgyroY : ");Serial.print(mpu6050.getGyroY());
    Serial.print("\tgyroZ : ");Serial.println(mpu6050.getGyroZ());

    Serial.print("accAngleX : ");Serial.print(mpu6050.getAccAngleX());
    Serial.print("\taccAngleY : ");Serial.println(mpu6050.getAccAngleY());

    Serial.print("gyroAngleX : ");Serial.print(mpu6050.getGyroAngleX());
    Serial.print("\tgyroAngleY : ");Serial.print(mpu6050.getGyroAngleY());
    Serial.print("\tgyroAngleZ : ");Serial.println(mpu6050.getGyroAngleZ());

    Serial.print("angleX : ");Serial.print(mpu6050.getAngleX());
    Serial.print("\tangleY : ");Serial.print(mpu6050.getAngleY());
    Serial.print("\tangleZ : ");Serial.println(mpu6050.getAngleZ());
    Serial.println("=======================================================\n");

    Serial.print(mpu6050.getGyroX());                                                                                                                           
    Serial.print(" ");
    Serial.print(mpu6050.getGyroY());
    Serial.print(" ");
    Serial.print(mpu6050.getGyroZ());
    Serial.println();
    */

    filter_ax.filter(mpu6050.getAccX()*9.81);
    filter_ay.filter(mpu6050.getAccY()*9.81);
    filter_az.filter((mpu6050.getAccZ()-1)*9.81);
    
    estimator.IMUAngularSpeedUpdate(mpu6050.getGyroX()*PI/180,mpu6050.getGyroX()*PI/180,mpu6050.getGyroX()*PI/180*0.0);
    estimator.IMULinearSpeedUpdate(filter_ax.getOutput(),filter_ay.getOutput(),filter_az.getOutput());
    
    //estimator.IMUAngularSpeedUpdate(mpu6050.getGyroX()*PI/180,mpu6050.getGyroX()*PI/180,mpu6050.getGyroX()*PI/180);
    //estimator.IMULinearSpeedUpdate(mpu6050.getAccX()*9.81,mpu6050.getAccY()*9.81,(mpu6050.getAccZ()-1)*9.81);
    Serial.print(estimator.get_u());
    Serial.print("\t");
    Serial.print(estimator.get_v());
    Serial.print("\t");
    Serial.println(estimator.get_w());
    /*Serial.print(mpu6050.getAccX()*9.81);
    Serial.print("\t");
    Serial.print(mpu6050.getAccY()*9.81);
    Serial.print("\t");
    Serial.println(mpu6050.getAccZ()*9.81);*/
    timer = millis();

  }

}
