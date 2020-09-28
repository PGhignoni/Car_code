#ifndef ESTIMATOR_H_INCLUDED
#define ESTIMATOR_H_INCLUDED

class Estimator{


    private:

    // update rate of the filter
        float m_dt;

    // linear speed in body frame
        float m_u;
        float m_v;
        float m_w;

    // angular speed in body frame
        float m_p;
        float m_q;
        float m_r;

    // quaternion
        float m_q0;
        float m_q1;
        float m_q2;
        float m_q3;


    public:

    // default constructor
    Estimator(float dt, float u=0.0, float v=0.0, float w=0.0, float p=0.0, float q=0.0, float r=0.0, float q0=1.0, float q1=0.0, float q2=0.0, float q3=0.0)
    : m_dt{dt}, m_u{u}, m_v{v}, m_w{w}, m_p{p}, m_q{q}, m_r{r}, m_q0{q0}, m_q1{q1}, m_q2{q2}, m_q3{q3}
    {
    }

    // linear speed getters
    float get_u();
    float get_v();
    float get_w();

    // angular speed setters
    float get_p();
    float get_q();
    float get_r();

    // attitude getters
    float get_q0();
    float get_q1();
    float get_q2();
    float get_q3();

    // angular speed updated
    void IMUAngularSpeedUpdate(float w_x, float w_y, float w_z);

    // linear speed integrator
    void IMULinearSpeedUpdate(float a_x, float a_y, float a_z);

    // kinematic integrator
    // void IMUAttitudeIntegrator(); // To be implemented by taking into account the quaternion-based attitude kinematic

}
;

#endif // ESTIMATOR_H_INCLUDED
