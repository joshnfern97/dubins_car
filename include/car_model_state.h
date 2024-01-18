#ifndef CAR_MODEL_HPP
#define CAR_MODEL_HPP

// Define a struct to hold the values
struct car_model_state {
    float v_des;
    float v;
    float omega_des;
    float omega;
    float phi_dot_r_des;
    float phi_dot_l_des;
    float phi_dot_r;
    float phi_dot_l;
    float phi_ddot_r;
    float phi_ddot_l;
    float u_r;
    float u_l;
    float phi_dot_r_error;
    float phi_dot_l_error;
    float dt;
};

#endif // MYSTRUCT_HPP