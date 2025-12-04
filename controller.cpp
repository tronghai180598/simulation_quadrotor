#include "controller.h"
#include <cmath>

// Helper struct for 3D vectors
struct Vec3 {
    float x, y, z;
};

Vec3 rotateGFtoBF(float PHI, float THETA, float PSI, float X, float Y, float Z) {
    float X_ = cos(PSI)*cos(THETA)*X + sin(PSI)*cos(THETA)*Y - sin(THETA)*Z;
    float Y_ = (cos(PSI)*sin(PHI)*sin(THETA) - cos(PHI)*sin(PSI))*X + (sin(PHI)*sin(PSI)*sin(THETA)+cos(PHI)*cos(PSI))*Y + (cos(THETA)*sin(PHI))*Z;
    float Z_ = (cos(PHI)*cos(PSI)*sin(THETA) + sin(PHI)*sin(PSI))*X + (cos(PHI)*sin(PSI)*sin(THETA)-cos(PSI)*sin(PHI))*Y + (cos(PHI)*cos(THETA))*Z;
    return {X_, Y_, Z_};
}
    
Vec3 rotateBFtoGF(float PHI, float THETA, float PSI, float X, float Y, float Z) {
    float X_ = cos(PSI)*cos(THETA)*X + (cos(PSI)*sin(PHI)*sin(THETA)-cos(PHI)*sin(PSI))*Y + (cos(PHI)*cos(PSI)*sin(THETA)+sin(PHI)*sin(PSI))*Z;
    float Y_ = sin(PSI)*cos(THETA)*X + (sin(PSI)*sin(PHI)*sin(THETA)+cos(PHI)*cos(PSI))*Y + (cos(PHI)*sin(PSI)*sin(THETA)-cos(PSI)*sin(PHI))*Z;
    float Z_ = -sin(THETA)*X + cos(THETA)*sin(PHI)*Y + cos(PHI)*cos(THETA)*Z;
    return {X_, Y_, Z_};
}

void Saturation (float& val, float min, float max){ val = (val < min)?min:(val>max?max:val); } 

Controller::Controller() {
    dt = 0.001;
}
Controller::Controller(float dt_) {
    dt = dt_;
}

void Controller::_pid_position() {
    Vec3 des_bf = rotateGFtoBF(0.0, 0.0, 0.0, x_des, y_des, z_des);
    Vec3 pos_bf = rotateGFtoBF(state[0], state[1], state[2], state[9], state[10], state[11]);
    Vec3 vel_bf = rotateGFtoBF(state[0], state[1], state[2], state[6], state[7], state[8]);
    
    float x_error = des_bf.x - pos_bf.x;
    float y_error = des_bf.y - pos_bf.y;
    float z_error = des_bf.z - pos_bf.z;

    x_error_sum += x_error * dt;
    theta_des = -(x_kp * x_error + x_ki * x_error_sum + x_kd * vel_bf.x);
        
    y_error_sum += y_error * dt;
    phi_des = x_kp * y_error + x_ki * y_error_sum + x_kd * vel_bf.y;
    z_error_sum += z_error * dt;
    float denominator = cos(state[0]) * cos(state[1]);
    F_total = (m * (g + (z_kp * z_error + 
                  z_ki * z_error_sum + 
                  z_kd * vel_bf.z))) / std::max(0.1f, denominator);
    Saturation (theta_des, -phi_max, phi_max);
    Saturation(phi_des, -phi_max, phi_max);
    Saturation(F_total, u1_min, u1_max);
}
void Controller::_pid_attitude() {
    float phi_error = phi_des - state[0];
    float theta_error = theta_des - state[1];
    p_des = phi_kp * phi_error;
    q_des = phi_kp * theta_error;
    r_des = 0.0;
    Saturation (p_des, -p_max, p_max);
    Saturation (q_des, -p_max, p_max);
}
void Controller::_pid_rate(){
    float p_error = p_des - state[3];
    float q_error = q_des - state[4];
    float r_error = r_des - state[5];

    p_error_sum += p_error * dt;
    q_error_sum += q_error * dt;
    r_error_sum += r_error * dt;

    tau_phi = p_kp * p_error + p_ki * p_error_sum + p_kd * (p_error - prev_p);
    tau_theta = phi_kp * q_error + p_ki * q_error_sum + p_kd * (q_error - prev_q);
    tau_psi = phi_kp * r_error + p_ki * r_error_sum + p_kd * (r_error - prev_r);

    Saturation (tau_phi, u2_min, u2_max);
    Saturation (tau_theta, u3_min, u3_max);
    Saturation (tau_psi, u4_min, u4_max);

    prev_p = p_error;
    prev_q = q_error;
    prev_r = r_error;

}
void Controller::update(){
    _pid_position();
    _pid_attitude();
    _pid_rate();   
}
