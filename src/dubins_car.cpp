#include "..\include\dubins_car.h"
#include "..\include\cartesian_v.h"
#include "..\include\car_model_state.h"
#include <Eigen/Dense>

#include <iostream>
#include <stdio.h>
#include <math.h> 
#include <algorithm>  



dubins_car::dubins_car(){

    std::cout << "Dubins Car Read to Drive" << std::endl;

};



cartesian_v dubins_car::get_cartesian_v(float velocity, float theta){

    car_cartesian_v.vel_x = velocity*cos(theta);
    car_cartesian_v.vel_y = velocity*sin(theta);
    return car_cartesian_v;

};

car_model_state dubins_car::car_model(car_model_state states){
    
    //used to find phi_ddot from phi_dot (experimentally determined)
    Eigen::MatrixXd A(2,2);
    A << -1/this->Tau, 0, 
          0, -1/this->Tau;

    //used to convert inputs (from PID controller) to phi_ddot (experimentally determined)
    Eigen::MatrixXd B(2,2);
    B << this->K/this->Tau, 0, 
          0, this->K/this->Tau;

    // used to convert phi_dot to velocity and twist of the robot (from robot geometry)
    Eigen::MatrixXd C(2,2);
    C << this->R/2, this->R/2, 
         this->R/(2*this->L), -1*this->R/(2*this->L);

    //find the desired wheel velocities from the desired velocity and twist of the robot (from robot geometry)
    states.phi_dot_r_des = (states.omega_des*this->L/this->R) + states.v_des/this->R;
    states.phi_dot_l_des = (states.v_des/this->R) - (states.omega_des*this->L/this->R);
    Eigen::MatrixXd x(2,1);

    //find error between desired and actual
    states.phi_dot_l_error = states.phi_dot_l_des - states.phi_dot_l;
    states.phi_dot_r_error = states.phi_dot_r_des - states.phi_dot_r;

    //find inputs based on error
    states = this->PID(states);

    
    
    x << states.phi_dot_r, 
          states.phi_dot_l;

    Eigen::MatrixXd u(2,1);
    u << states.u_r, 
          states.u_l;

    Eigen::MatrixXd x_dot(2,1);
    Eigen::MatrixXd y(2,1);

    //perform state space multiplication
    //x_dot gives us wheel angular acceleration
    //y gives us velocity and twist of the robot
    x_dot = A*x + B*u;
    y = C*x;

    states.v = y(0,0);
    states.omega = y(1,0);


    //make sure the phi_ddot_r does not exceed the max
    if (abs(x_dot(0,0)) < this->phi_ddot_max){
        states.phi_ddot_r = x_dot(0,0);
    }
    else{
        if (x_dot(0,0) < 0){
            states.phi_ddot_r = -1*this->phi_ddot_max;
        }else{
            states.phi_ddot_r = this->phi_ddot_max;
        }

    }

    //make sure the phi_ddot_l does not exceed the max
    if (abs(x_dot(1,0)) < this->phi_ddot_max){
        states.phi_ddot_l = x_dot(1,0);
    }
    else{
        if (x_dot(1,0) < 0){
            states.phi_ddot_l = -1*this->phi_ddot_max;
        }else{
            states.phi_ddot_l = this->phi_ddot_max;
        }

    }

    //Update states and make sure it does not exceed max
    states.phi_dot_r = states.phi_dot_r + states.phi_ddot_r*this->dt;
    if (abs(states.phi_dot_r) > this->phi_dot_max){
        std::cout << "RIGHT WHEEL AT MAX SPEED" << std::endl;
        states.phi_dot_r = std::max(-1*this->phi_dot_max, std::min(this->phi_dot_max, states.phi_dot_r));
    }

    states.phi_dot_l = states.phi_dot_l + states.phi_ddot_l*this->dt;
    if (abs(states.phi_dot_l) > this->phi_dot_max){
        std::cout << "LEFT WHEEL AT MAX SPEED" << std::endl;
        states.phi_dot_l = std::max(-1*this->phi_dot_max, std::min(this->phi_dot_max, states.phi_dot_l));
    }
    
    
    return states;
}

car_model_state dubins_car::PID(car_model_state states){
    
    //Left wheel PID controller
    this->pterm_l = this->p_l*states.phi_dot_l_error;
    this->integral_l = states.phi_dot_l_error*this->dt;
    this->integral_l = std::max(-1*this->i_max_l, std::min(this->i_max_l, this->integral_l));
    this->iterm_l = this->i_l*this->integral_l;
    this->dterm_l = this->d_l*((states.phi_dot_l_error - this->previous_l_error)/this->dt);

    //Right wheel PID controller
    this->pterm_r = this->p_r*states.phi_dot_r_error;
    this->integral_r = states.phi_dot_r_error*this->dt;
    this->integral_r = std::max(-1*this->i_max_r, std::min(this->i_max_r, this->integral_r));
    this->iterm_r = this->i_r*this->integral_r;
    this->dterm_r = this->d_r*((states.phi_dot_r_error - this->previous_r_error)/this->dt);

    //store previous error for derivative term
    this->previous_l_error = states.phi_dot_l_error;
    this->previous_r_error = states.phi_dot_r_error;

    //update desired inputs
    states.u_l = this->pterm_l + this->iterm_l + this->dterm_l;
    states.u_r = this->pterm_r + this->iterm_r + this->dterm_r;

    return states;
}

void dubins_car::SayHello(){
    std::cout<<"Hello from the car!"<<std::endl;
}