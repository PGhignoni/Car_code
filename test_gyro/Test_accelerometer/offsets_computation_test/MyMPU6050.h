#ifndef MYMPU6050_H_INCLUDED
#define MYMPU6050_H_INCLUDED

#include <MPU6050_tockn.h>

class MyMPU6050: public MPU6050{

    protected:
	// non-raw accelerometer biases
        float accOffsetX{0.0};
        float accOffsetY{0.0};
        float accOffsetZ{0.0};
  	
	// non-raw accelerometer standard deviation
        float sigmaX{0.0};
        float sigmaY{0.0};
        float sigmaZ{0.0};

    public:

        MyMPU6050(TwoWire &w): MPU6050{w}
        {
        }

	// method which computes the accelerometer offset and standard deviation of the measurement noise of the accelerometer
        void computeAccOffset(bool console=false, uint16_t delayBefore=1000, uint16_t delayAfter=3000);

	// method which is used to update the measurements of the gyro(remove the constant offset to the in plane acceleration and rates)
        void update();

	// method which is used to extract the standard deviation of the noise (mean value of the noise on the three channels)
	float getAccelerometerNoiseSigma();
};


#endif // MYMPU6050_H_INCLUDED
