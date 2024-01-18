#include <iostream>
#include <Eigen/Dense>
#include "..\include\constants.h"
#include "..\include\matplotlibcpp.h"
#include "..\include\dubins_car.h"
#include "..\include\PID.h"
#include "..\include\car_model_state.h"
#include <conio.h>
#include <Windows.h>

 
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace plt = matplotlibcpp;

int main()
{
    //Intialize Car States
    std::vector<float> x_car_pos(1,0);
    std::vector<float> y_car_pos(1,0);
    float desired_theta = 0;
    float theta = 0;
    int quit = 0;

    //Unit vector used to plot robot/human coordinate systems
    Eigen::MatrixXd x_unit(4,1);
    x_unit << 2, 0, 0, 1;
    Eigen::MatrixXd y_unit(4,1);
    y_unit << 0, 2, 0, 1;

    //Intialize car pose - (x, y, heading)
    Eigen::MatrixXd car_pos_array(4,1);
    car_pos_array << 0, 0, 0, 1;

    //Define car geometry
    Eigen::MatrixXd right_wheel_pos(4,1);
    right_wheel_pos << 0, -1*L, 0, 1;
    Eigen::MatrixXd left_wheel_pos(4,1);
    left_wheel_pos << 0, L, 0, 1;
    Eigen::MatrixXd car_pos(4,1);
    car_pos << 0, 0, 0, 0;
    Eigen::MatrixXd T_world_car(4,4);
    T_world_car << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1;
    

    Eigen::MatrixXd x_frame_car(4,1);
    Eigen::MatrixXd y_frame_car(4,1);
    Eigen::MatrixXd r_wheel_in_world(4,1);
    Eigen::MatrixXd l_wheel_in_world(4,1);

    float x_frame_car_tip_x;
    float x_frame_car_tip_y;

    float y_frame_car_tip_x;
    float y_frame_car_tip_y;

    float r_wheel_x;
    float r_wheel_y;
    float l_wheel_x;
    float l_wheel_y;
    
    //define car and states
    dubins_car car;
    cartesian_v car_cartesian_v;
    car_model_state states;
    float velocity = 0;
    float desired_velocity = 0;
    states.v = velocity;
    states.v_des = desired_velocity;
    states.phi_ddot_l = 0;
    states.phi_ddot_r = 0;
    states.phi_dot_l = 0;
    states.phi_dot_r = 0;
    states.u_r = 0;
    states.u_l = 0;
    states.omega = 0;
    states.target_error_d = 0;
    states.target_error_x = 0;

    //marker that moves around the space that the robot follows
    std::vector<float> marker_x(1,0);
    std::vector<float> marker_y(1,0);


    //define person states
    Eigen::MatrixXd person_pos(4,1);
    person_pos << marker_x[0], marker_y[0], 0, 1;
    Eigen::MatrixXd person_rel_car(4,1);
    float x_error = 0;
    float d_error = 0;
    float d_desired = 0;


    ////////////////////////////////////
    //Setup High-level PID controller //
    ////////////////////////////////////
    //linear PID
    float p_lin = 5;
    float i_lin = 1;
    float d_lin = 0.05;
    float i_max_lin = 1;
    float previous_error_lin = 0;

    //angular PID
    float p_ang = 7;
    float i_ang = 1;
    float d_ang = 0.05;
    float i_max_ang= 1;
    float previous_error_ang = 0;

    float V_max = 3;
    float omega_max = 3;

    PID pid(p_lin, i_lin, d_lin, i_max_lin, p_ang, i_ang, d_ang, i_max_ang);    
    
    while (quit == 0){
        //Look for input from user
        char key = ' ';
        if (_kbhit())
        {
            key = _getch();
            if (key == 'w')
                {
                    std::cout << "Moving Up" << std::endl;
                    marker_y[0] += 0.1;
                }

            if (key == 'x')
                {
                    std::cout << "Moving Down" << std::endl;
                    marker_y[0] -= 0.1;
                }
            if (key == 'a')
                {
                    std::cout << "Moving Left" << std::endl;
                    marker_x[0] -= .1;
                }
            if (key == 'd')
                {
                    std::cout << "Turn Right" << std::endl;
                    marker_x[0] += .1;
                }
            if (key == 'q')
                {
                    std::cout << "Quitting Program" << std::endl;

                    quit = 1;
            }
           
        }

        //updated marker/person position
        person_pos << marker_x[0], marker_y[0], 0, 1;
        //Find the person (marker) relative to the car
        person_rel_car = T_world_car.inverse()*person_pos;
       
        x_error = person_rel_car(1);
        d_error = person_rel_car(0);
        states.target_error_d = d_error;
        states.target_error_x = x_error;
        //d_error = person_rel_car[0] - desired_d; //If you want the robot to follow behind the marker
        std::cout << "X Error of the person: " << x_error << std::endl;
        std::cout << "D Error to the person: " << d_error << std::endl;
        std::cout << "----------------------------------------" << std::endl;

        //gets the desired velocity and twist commands from the highlevel pid controller
        states = pid.get_desired(states);
        //Make sure the desired velocity and twist are not greater than the max
        if (abs(states.v_des) > V_max){
            states.v_des = std::max(-1*V_max, std::min(V_max, states.v_des));
        }
        if (abs(states.omega_des) > omega_max){
            states.omega_des = std::max(-1*omega_max, std::min(omega_max, states.omega_des));
        }

        //get the true states of the car from the car model
        states = car.car_model(states);
        
        // If you want the car model values as true values
        velocity = states.v;
        theta = theta + states.omega*dt;


        //get the car position in x-y coordinates
        car_cartesian_v = car.get_cartesian_v(velocity, theta);

        //car position
        car_pos << x_car_pos[0], y_car_pos[0], 0, 1;

        //Homogenous Transformation Matrix
        T_world_car << cos(theta), -1*sin(theta), 0, x_car_pos[0],
                       sin(theta),    cos(theta), 0, y_car_pos[0],
                       0, 0, 1, 0,
                       0, 0, 0, 1;  

        //Rotate axes
        x_frame_car = T_world_car*x_unit;
        y_frame_car = T_world_car*y_unit;

        //Rotate wheels
        r_wheel_in_world = T_world_car*right_wheel_pos;
        l_wheel_in_world = T_world_car*left_wheel_pos;
        
        
        //get coordinates of axes for plotting
        x_frame_car_tip_x = x_frame_car(0,0);
        x_frame_car_tip_y = x_frame_car(1,0);
        y_frame_car_tip_x = y_frame_car(0,0);
        y_frame_car_tip_y = y_frame_car(1,0);

        std::vector<float> x_axis_x = {x_car_pos[0], x_frame_car_tip_x};
        std::vector<float> x_axis_y = {y_car_pos[0], x_frame_car_tip_y};
        std::vector<float> y_axis_x = {x_car_pos[0], y_frame_car_tip_x};
        std::vector<float> y_axis_y = {y_car_pos[0], y_frame_car_tip_y};
        
        //get coordinates of wheels for plotting
        r_wheel_x = r_wheel_in_world(0,0);
        r_wheel_y = r_wheel_in_world(1,0);
        l_wheel_x = l_wheel_in_world(0,0);
        l_wheel_y = l_wheel_in_world(1,0);
        std::vector<float> r_wheel_in_world_x = {r_wheel_x};
        std::vector<float> r_wheel_in_world_y = {r_wheel_y};
        std::vector<float> l_wheel_in_world_x = {l_wheel_x};
        std::vector<float> l_wheel_in_world_y = {l_wheel_y};


        //update car position
        x_car_pos[0] = x_car_pos[0] + car_cartesian_v.vel_x*dt;
        y_car_pos[0] = y_car_pos[0] + car_cartesian_v.vel_y*dt;
        
        //visualize
        plt::clf();
  
        
        plt::plot(x_axis_x, x_axis_y, "r");
        plt::plot(y_axis_x, y_axis_y, "g");
        plt::plot(r_wheel_in_world_x, r_wheel_in_world_y, "cs");
        plt::plot(l_wheel_in_world_x, l_wheel_in_world_y, "bs");
        plt::plot(x_car_pos, y_car_pos, "ro");
        plt::plot(marker_x, marker_y, "rx");
        plt::xlim(-10, 10);
        plt::ylim(-10, 10);
        
        plt::pause(0.01);
    }


}