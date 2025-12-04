#include "Quadrotor.h"
class Controller : public Quadrotor {
public:
    Controller();
    Controller(float dt);
    float x_des = 0.0f;
    float y_des = 0.0f;
    float z_des = 0.0f;
    float phi_des = 0.0f;
    float theta_des = 0.0f;
    float psi_des = 0.0f;
    float p_des = 0.0f;
    float q_des = 0.0f;
    float r_des = 0.0f;

    float x_kp = 0.35;
    float x_ki = 0.0f;
    float x_kd = -0.35;
    float z_kp = 4.88;
    float z_ki = 0.0f;
    float z_kd = -3.0f;
    float phi_kp = 4.5;
    float p_kp = 2.7;
    float p_ki = 0.0f;
    float p_kd = 0.05f;
    float x_error_sum = 0.0f;
    float y_error_sum = 0.0f;
    float z_error_sum = 0.0f;
    float p_error_sum = 0.0f;
    float q_error_sum = 0.0f;
    float r_error_sum = 0.0f;

    float u1_max = 100.0f;
    float u1_min = 0.0f;
    float u2_max = 50.0f;
    float u2_min = -50.0f;
    float u3_max = 50.0f;
    float u3_min = -50.0f;
    float u4_max = 50.0f;
    float u4_min = -50.0f;
    float prev_p = 0.0f;
    float prev_q = 0.0f;
    float prev_r = 0.0f;
    float F_total = 0.0f;
    float tau_phi = 0.0f;
    float tau_theta = 0.0f;
    float tau_psi = 0.0f;

    float x_des_bf, y_des_bf, z_des_bf;
    float x_bf, y_bf, z_bf;
    float x_bf_dot, y_bf_dot, z_bf_dot;
    
    // float Kpp_rate = 1.0f;       // Increased from 0.5
    // float Kip_rate = 0.1f;       // Increased from 0.05
    
    // float Kpr_rate = 4.0f;       // Reduced from 0.5
    // float Kir_rate = 0.05f;      // Reduced from 0.1

    // float intergral_Vp = 0.0f; // Placeholder for integral term
    // float intergral_Vq = 0.0f; // Placeholder for integral term
    // float intergral_Vr = 0.0f; // Placeholder for integral term

    // float tau_phi = 0.0f;
    // float tau_theta = 0.0f;
    // float tau_psi = 0.0f;

    void _pid_position();
    void _pid_attitude();
    void _pid_rate();
    void update();
};