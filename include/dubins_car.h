#include "cartesian_v.h"
#include "car_model_state.h"
#include <Eigen/Dense>


class dubins_car
{
    private:
        cartesian_v car_cartesian_v; 
        
        float R = 0.254;
        float L = 0.1905;

        float phi_dot_max = 12;
        float phi_ddot_max = 54;

        float Tau = 0.25;
        float K = 0.83;

        

        float p_r = 20;
        float i_r = 10;
        float d_r = 0;
        float i_max_r =5;

        float p_l = 20;
        float i_l = 10;
        float d_l = 0;
        float i_max_l = 1;
        float dt = 0.01;

        float pterm_l;
        float integral_l;
        float iterm_l;
        float dterm_l;
        float previous_l_error = 0;

        float pterm_r;
        float integral_r;
        float iterm_r;
        float dterm_r;
        float previous_r_error = 0;

        


    public:
        dubins_car();
        cartesian_v get_cartesian_v(float velocity, float theta);
        car_model_state car_model(car_model_state states);
        car_model_state PID(car_model_state states);
        void SayHello();
};