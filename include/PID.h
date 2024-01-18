#include "cartesian_v.h"
#include "car_model_state.h"
#include <Eigen/Dense>


class PID
{
    private:

        //linear PID
        float p_lin;
        float i_lin;
        float d_lin;
        float i_max_lin;
        float previous_error_lin = 0;

        float pterm_lin;
        float integral_lin;
        float iterm_lin;
        float dterm_lin;


        //angular PID
        float p_ang;
        float i_ang;
        float d_ang;
        float i_max_ang;
        float previous_error_ang = 0;

        float pterm_ang;
        float integral_ang;
        float iterm_ang;
        float dterm_ang;
        

        float dt = 0.01;

    public:
        PID(float p_lin, float i_lin, float d_lin, float i_max_lin, float p_ang, float i_ang, float d_ang, float i_max_ang);
        car_model_state get_desired(car_model_state states);
        
};