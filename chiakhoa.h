
#include "vector.h"
#include <vector>
class Quadrotor {
public: 
    Quadrotor(float dt);
    Quadrotor();
    float dt = 0.001;
    float time_elapse = 0.0;
    float g =  9.81;
    float m = 0.8;
    float l = 0.1;
    float kdx = 0.16481;
    float kdy = 0.16481;
    float kdz = 0.000011;
    float jx = 0.075;
    float jy = 0.075;
    float jz = 0.14;
    float kt = 0.000013328;
    float kd = 0.0000026656;
    float jp = 0.044;
    float p_max = 100.0;
    float phi_max= 90.0;
    float psi_max= 180.0;
    std::vector<float> state;  // x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, p, q, r
    std::vector<float> Forces;
    std::vector<float> motor_speeds;  // 4 motor speeds
    Vector control_inputs;
    void _calculate_translational_dynamics(float phi, float theta, float psi, float x_dot, float y_dot, float z_dot);              
    void _calculate_rotational_dynamics(float p, float q, float r);
    void _integrate_states();
    void _calculate_motor_speeds(std::vector<float> control_inputs);
    std::vector<float> step(float dt, std::vector<float> control_inputs);


  


    // float update(float dt, float inp);
    // float controller(float dt, float setF, float F, float V);

    // float getFi();
    // float getVi();
    // float getAcc();
    // void setPara();
};
