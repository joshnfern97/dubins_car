#include <iostream>
#include <Eigen/Dense>
#include "..\include\constants.h"
#include "..\include\matplotlibcpp.h"
#include "..\include\dubins_car.h"
#include "..\include\car_model_state.h"
#include <conio.h>
#include <Windows.h>

 
using Eigen::MatrixXd;
using Eigen::VectorXd;
// using Eigen::Matrix;
/*
When compiling:
g++ -I C:\Users\jfernandez75\Documents\eigen-3.4.0 test_eigen.cpp -o test_eigen
*/
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

    
    while (quit == 0){
        //Look for input from user
        char key = ' ';
        if (_kbhit())
        {
            key = _getch();
            if (key == 'w')
                {
                    std::cout << "Speeding up (" << desired_velocity <<" m/s)" << std::endl;
                    desired_velocity += 0.1;
                }

            if (key == 's')
                {
                    std::cout << "Stop Vehicle" << std::endl;
                    desired_velocity = 0;
                    desired_theta = 0;
                }
            if (key == 'x')
                {
                    std::cout << "Reverse  (" << desired_velocity <<" m/s)" << std::endl;
                    desired_velocity -= 0.1;
                }
            if (key == 'a')
                {
                    std::cout << "Turn Left" << std::endl;
                    desired_theta += .1;
                }
            if (key == 'd')
                {
                    std::cout << "Turn Right" << std::endl;
                    desired_theta -= .1;
                }
            if (key == 'q')
                {
                    std::cout << "Quitting Program" << std::endl;

                    quit = 1;
            }
            if (key == 'e')
                {
                    std::cout << "Stop Turning" << std::endl;

                    desired_theta = 0;
            }
        }

        //get the true states of the car from the car model
        states.v_des = desired_velocity;
        states.omega_des = desired_theta;
        states = car.car_model(states);
        
        // If you want the car model values as true values
        velocity = states.v;
        theta = theta + states.omega*dt;

        
        // // If you want the desired values to equal the true values
        // theta = desired_theta;
        // velocity = desired_velocity;

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
        plt::xlim(-10, 10);
        plt::ylim(-10, 10);
        
        plt::pause(0.01);
    }






}