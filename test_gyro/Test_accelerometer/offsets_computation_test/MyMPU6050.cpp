#include "MyMPU6050.h"
#include <math.h>

void MyMPU6050::computeAccOffset(bool console, uint16_t delayBefore, uint16_t delayAfter){
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;
	float xx{0.};
	float yy{0.};
	float zz{0.};

  delay(delayBefore);
	if(console){
    Serial.println();
    Serial.println("========================================");
    Serial.println("Calculating accelerometer offsets");
    Serial.print("DO NOT MOVE MPU6050");
  }
  for(int i = 0; i < 3000; i++){
    if(console && i % 1000 == 0){
      Serial.print(".");
    }
 	wire->beginTransmission(MPU6050_ADDR);
	wire->write(0x3B);
	wire->endTransmission(false);
	wire->requestFrom((int)MPU6050_ADDR, 14);

    rx = wire->read() << 8 | wire->read();
    ry = wire->read() << 8 | wire->read();
    rz = wire->read() << 8 | wire->read();

    x += ((float)rx) / 16384.0;
    y += ((float)ry) / 16384.0;
    z += ((float)rz) / 16384.0;

  }
  this->accOffsetX = x / 3000;
  this->accOffsetY = y / 3000;
  this->accOffsetZ = z / 3000;

  // computation of the std deviation

  float sumX{0.}, sumY{0.}, sumZ{0.};
  
  for(int i{0}; i<3000;i++){	
  if(console && i % 1000 == 0){
      Serial.print(".");
    }
  wire->beginTransmission(MPU6050_ADDR);
  wire->write(0x3B);
  wire->endTransmission(false);
  wire->requestFrom((int)MPU6050_ADDR, 14);

  rx = wire->read() << 8 | wire->read();
  ry = wire->read() << 8 | wire->read();
  rz = wire->read() << 8 | wire->read();

  // writing the data on the array
  xx=((float)rx) / 16384.0;
  yy=((float)ry) / 16384.0;
  zz=((float)rz) / 16384.0;
    
	sumX=sumX+(xx-this->accOffsetX)*(xx-this->accOffsetX);
	sumY=sumY+(yy-this->accOffsetY)*(yy-this->accOffsetY);
	sumZ=sumZ+(zz-this->accOffsetZ)*(zz-this->accOffsetZ);
  }

  this->sigmaX=sqrt(sumX/3000);
  this->sigmaY=sqrt(sumY/3000);
  this->sigmaZ=sqrt(sumZ/3000);

  // print on the console
  if(console){
    Serial.println();
    Serial.println("Done!");
    Serial.print("X bias: ");Serial.println(accOffsetX);
    Serial.print("Y bias: ");Serial.println(accOffsetY);
    Serial.print("Z bias: ");Serial.println(accOffsetZ);
    Serial.print("X noise: ");Serial.println(sigmaX, 6);
    Serial.print("Y noise: ");Serial.println(sigmaY, 6);
    Serial.print("Z noise: ");Serial.println(sigmaZ, 6);
    Serial.println("Program will start after 3 seconds");
    Serial.print("========================================");
		delay(delayAfter);
	}

}


void MyMPU6050::update(){
	wire->beginTransmission(MPU6050_ADDR);
	wire->write(0x3B);
	wire->endTransmission(false);
	wire->requestFrom((int)MPU6050_ADDR, 14);

  rawAccX = wire->read() << 8 | wire->read();
  rawAccY = wire->read() << 8 | wire->read();
  rawAccZ = wire->read() << 8 | wire->read();
  rawTemp = wire->read() << 8 | wire->read();
  rawGyroX = wire->read() << 8 | wire->read();
  rawGyroY = wire->read() << 8 | wire->read();
  rawGyroZ = wire->read() << 8 | wire->read();

  temp = (rawTemp + 12412.0) / 340.0;

// Remove the offset
  accX = ((float)rawAccX) / 16384.0-this->accOffsetX;//<----------------------------------------------------------- New offset remove
  accY = ((float)rawAccY) / 16384.0-this->accOffsetY;//<----------------------------------------------------------- New offset remove
  accZ = ((float)rawAccZ) / 16384.0;//<----------------------------------------------------------- New offset remove (unnecessary for the moment)

  angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
  angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;

  gyroX = ((float)rawGyroX) / 65.5;
  gyroY = ((float)rawGyroY) / 65.5;
  gyroZ = ((float)rawGyroZ) / 65.5;

  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;

  interval = (millis() - preInterval) * 0.001;

  angleGyroX += gyroX * interval;
  angleGyroY += gyroY * interval;
  angleGyroZ += gyroZ * interval;

  angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;

  preInterval = millis();

}


float MyMPU6050::getAccelerometerNoiseSigma(){

	float meanNoise{(this->sigmaX+this->sigmaY+this->sigmaZ)/3};
	return meanNoise;

}

