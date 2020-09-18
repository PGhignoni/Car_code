#ifndef MYMPU6050_H_INCLUDED
#define MYMPU6050_H_INCLUDED

#include <MPU6050_tockn.h>

class MyMPU6050: public MPU6050{

    protected:
	// non-raw accelerometer biases
        float accOffsetX{0.0};
        float accOffsetY{0.0};
        float accOffsetZ{0.0};

    public:

        MyMPU6050(TwoWire &w): MPU6050{w}
        {
        }
        void computeAccOffset(bool console=false, uint16_t delayBefore=1000, uint16_t delayAfter=3000);
        void update();
};


#endif // MYMPU6050_H_INCLUDED
