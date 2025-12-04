#include "Quadrotor.h"
#include <string>
#include <vector>
#include <math.h>
#include <iostream>

void satura(float& val, float min, float max){ val = (val < min)?min:(val>max?max:val); } 

Quadrotor::Quadrotor(float dt_){
  dt = dt_;
}
Quadrotor::Quadrotor(){
}
void Quadrotor::_calculate_translational_dynamics(float phi, float theta, float psi, float x_dot, float y_dot, float z_dot){
        float R_x = (cos(phi)*sin(theta)*cos(psi) + sin(psi)*sin(phi));
        float R_y = (cos(phi)*sin(psi)*sin(theta) - cos(psi)*sin(phi));
        float R_z = cos(phi)*cos(theta);
        
        state[6] += (-R_x * Forces[0]/m - kdx*x_dot/m)* dt;  //ddot x
        state[7] += (-R_y * Forces[0]/m - kdy*y_dot/m) * dt; //ddot y
        state[8] += ((R_z * Forces[0]/m) - g - (kdz*z_dot/m)) * dt; //ddot z
}
void Quadrotor::_calculate_rotational_dynamics(float p, float q, float r){
        float p_dot = (q*r*(jy - jz) + Forces[1]) / jx; // ddot p
        float q_dot = (p*r*(jz - jx) + Forces[2]) / jy;  // ddot q
        float r_dot = (p*q*(jx - jy) + Forces[3]) / jz;  // ddot r
        state[3] += p_dot * dt;   // p
        state[4] += q_dot * dt;   // q
        state[5] += r_dot * dt;   // r
        satura(state[3], -p_max, p_max);
        satura(state[4], -p_max, p_max);
        satura(state[5], -p_max, p_max);
}
void Quadrotor::_integrate_states(){
        float phi = state[0];
        float theta = state[1];
        float psi = state[2];
        float p = state[3];
        float q = state[4];
        float r = state[5];

        phi_dot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;  // dot phi
        theta_dot = cos(phi)*q - sin(phi)*r;  // dot theta
        psi_dot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;  // dot psi
        state[0] += phi_dot * dt;  // phi
        state[1] += theta_dot * dt;  // theta
        state[2] += psi_dot * dt;  // psi
        satura(state[0], -phi_max, phi_max);
        satura(state[1], -phi_max, phi_max);
        satura(state[2], -psi_max, psi_max);

        state[9] += state[6] * dt; // x position
        state[10] += state[7] * dt; // y position
        state[11] = std::max(0.0f, state[11] + state[8] * dt); // z position
}
void Quadrotor::_calculate_motor_speeds(std::vector<float> control_inputs){
        float F_total = control_inputs[0];
        float tau_phi = control_inputs[1];
        float tau_theta = control_inputs[2];
        float tau_psi = control_inputs[3];
        
        // Calculate motor speeds using inverse of mixing matrix
        std::vector<float> speeds = {
            (F_total/kt + tau_phi/(l*kt) + tau_theta/(l*kt) + tau_psi/kd) / 4.0f,
            (F_total/kt + tau_phi/(l*kt) - tau_theta/(l*kt) - tau_psi/kd) / 4.0f,
            (F_total/kt - tau_phi/(l*kt) - tau_theta/(l*kt) + tau_psi/kd) / 4.0f,
            (F_total/kt - tau_phi/(l*kt) + tau_theta/(l*kt) - tau_psi/kd) / 4.0f
        };
        
        // Store in member variable
        motor_speeds = speeds;
        Forces[0] = kt * (motor_speeds[0] + motor_speeds[1] + motor_speeds[2] + motor_speeds[3]);
        Forces[1] = kt * l * (motor_speeds[0] + motor_speeds[1] - motor_speeds[2] - motor_speeds[3]);
        Forces[2] = kt * l * (motor_speeds[0] - motor_speeds[1] - motor_speeds[2] + motor_speeds[3]);
        Forces[3] = kd * (motor_speeds[0] - motor_speeds[1] + motor_speeds[2] - motor_speeds[3]);
}
std::vector<float> Quadrotor::step(float dt, std::vector<float> control_inputs){
        _calculate_motor_speeds(control_inputs);
        float phi = state[0];
        float theta = state[1];
        float psi = state[2];
        float p = state[3];
        float q = state[4];
        float r = state[5];
        float x_dot = state[6];
        float y_dot = state[7];
        float z_dot = state[8];
        _calculate_translational_dynamics( phi, theta, psi, x_dot, y_dot, z_dot);
        _calculate_rotational_dynamics( p, q, r);
        _integrate_states();
    // state = state + control_inputs * dt;
        time_elapse += dt;
        return state;
}




