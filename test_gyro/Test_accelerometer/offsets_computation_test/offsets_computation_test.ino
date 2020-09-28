#include "MyMPU6050.h"
#include <Wire.h>

MyMPU6050 mpu6050(Wire);



void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  mpu6050.computeAccOffset(true);
}

void loop() {
  // put your main code here, to run repeatedly:

}
