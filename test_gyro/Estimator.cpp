# include "Estimator.h"


float Estimator::get_u(){
    return this->m_u;
}

float Estimator::get_v(){
    return this->m_v;
}

float Estimator::get_w(){
    return this->m_w;
}

float Estimator::get_p(){
    return this->m_p;
}

float Estimator::get_q(){
    return this->m_q;
}

float Estimator::get_r(){
    return this->m_r;
}

float Estimator::get_q0(){
    return this->m_q0;
}

float Estimator::get_q1(){
    return this->m_q1;
}

float Estimator::get_q2(){
    return this->m_q2;
}

float Estimator::get_q3(){
    return this->m_q3;
}

void Estimator::IMUAngularSpeedUpdate(float w_x, float w_y, float w_z){
    this->m_p=w_x;
    this->m_q=w_y;
    this->m_r=w_z;
}

void Estimator::IMULinearSpeedUpdate(float a_x, float a_y, float a_z){

    float x{this->m_dt*this->m_r};
    float det{1+x*x};

    float a{this->m_dt*a_x+this->m_u};
    float b{this->m_dt*a_y+this->m_v};

    this->m_u=(a+x*b)/det;
    this->m_v=(-a*x+b)/det;
    this->m_w=a_z*this->m_dt+this->m_w;
}

