#include "..\include\PID.h"
#include "..\include\cartesian_v.h"
#include "..\include\car_model_state.h"
#include <Eigen/Dense>

#include <iostream>
#include <stdio.h>
#include <math.h> 
#include <algorithm>  



PID::PID(float p_lin, float i_lin, float d_lin, float i_max_lin, float p_ang, float i_ang, float d_ang, float i_max_ang){
    
    this->p_lin = p_lin;
    this->i_lin = i_lin;
    this->d_lin = d_lin;
    this->i_max_lin = i_max_lin;

    this->p_ang = p_ang;
    this->i_ang = i_ang;
    this->d_ang = d_ang;
    this->i_max_ang = i_max_ang;

    std::cout << "PID SETUP COMPLETE" << std::endl;

};



car_model_state PID::get_desired(car_model_state states){
    
    //linear PID 
    this->pterm_lin = this->p_lin*states.target_error_d;
    this->integral_lin = states.target_error_d*this->dt;
    this->integral_lin = std::max(-1*this->i_max_lin, std::min(this->i_max_lin, this->integral_lin));
    this->iterm_lin = this->i_lin*this->integral_lin;
    this->dterm_lin = this->d_lin*((states.target_error_d - this->previous_error_lin)/this->dt);

    //Angular PID
    this->pterm_ang = this->p_ang*states.target_error_x;
    this->integral_ang = states.target_error_x*this->dt;
    this->integral_ang = std::max(-1*this->i_max_ang, std::min(this->i_max_ang, this->integral_ang));
    this->iterm_ang = this->i_ang*this->integral_ang;
    this->dterm_ang = this->d_ang*((states.target_error_x - this->previous_error_ang)/this->dt);

    //store previous error for derivative term
    this->previous_error_lin = states.target_error_d;
    this->previous_error_ang = states.target_error_x;

    //update desired velocity and twist
    states.v_des = this->pterm_lin + this->iterm_lin + this->dterm_lin;
    states.omega_des = this->pterm_ang + this->iterm_ang + this->dterm_ang;


    return states;

}