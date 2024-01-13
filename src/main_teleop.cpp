#include <iostream>
#include <Eigen/Dense>
#include "..\include\constants.h"
#include "..\include\matplotlibcpp.h"
#include "..\include\dubins_car.h"
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
    std::vector<float> phi_dot_r(1,0);
    std::vector<float> phi_dot_r_des(1,0);
    std::vector<float> phi_dot_r_error(1,0);
    std::vector<float> phi_dot_l(1,0);
    std::vector<float> phi_dot_l_des(1,0);
    std::vector<float> phi_dot_l_error(1,0);
    std::vector<float> u_r(1,0);
    std::vector<float> u_l(1,0);
    std::vector<float> phi_ddot_r(1,0);
    std::vector<float> phi_ddot_l(1,0);
    std::vector<float> v(1,0);
    std::vector<float> omega(1,0);
    std::vector<float> time(1,0);
    std::vector<float> x_pos(1,0);
    // std::vector<float> theta_pos(1,0);
    std::vector<float> theta_pos(1,3.14/2);
    std::vector<float> car_v_des(1,0);
    std::vector<float> car_omega_des(1,0);
    std::vector<float> robot_compass_heading_array(1,0);
    std::vector<float> x_car_pos(1,0);
    std::vector<float> y_car_pos(1,0);


   
    // float x_car_pos = 0;
    // float y_car_pos = 0;
    float V_desired = 0;
    float u_desired = 0;

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
    
    dubins_car car;
    cartesian_v car_cartesian_v;
    float velocity = 0;
    float theta = 0;
    int quit = 0;
    while (quit == 0){
        
        char key = ' ';
        if (_kbhit())
        {
            key = _getch();
            if (key == 'w')
                {
                    std::cout << "Speeding up (" << velocity <<" m/s)" << std::endl;
                    velocity += 0.1;
                }

            if (key == 's')
                {
                    std::cout << "Stop Vehicle" << std::endl;
                    velocity = 0;
                }
            if (key == 'x')
                {
                    std::cout << "Reverse  (" << velocity <<" m/s)" << std::endl;
                    velocity -= 0.1;
                }
            if (key == 'a')
                {
                    std::cout << "Turn Left" << std::endl;
                    theta += .1;
                }
            if (key == 'd')
                {
                    std::cout << "Turn Right" << std::endl;
                    theta -= .1;
                }
            if (key == 'q')
                {
                    std::cout << "Quitting Program" << std::endl;

                    quit = 1;
            }
        }
        car_cartesian_v = car.get_cartesian_v(velocity, theta);

        // std::cout << "Y Velocity: " << car_cartesian_v.vel_y << std::endl;

        // time.push_back(time[i-1]+dt);
        // theta_pos.push_back(theta_pos[i-1]+ .01);

        car_pos << x_car_pos[0], y_car_pos[0], 0, 1;

        T_world_car << cos(theta), -1*sin(theta), 0, x_car_pos[0],
                       sin(theta),    cos(theta), 0, y_car_pos[0],
                       0, 0, 1, 0,
                       0, 0, 0, 1;  

        x_frame_car = T_world_car*x_unit;
        y_frame_car = T_world_car*y_unit;

        // std::cout << "Theta: " << theta << std::endl;
        
        r_wheel_in_world = T_world_car*right_wheel_pos;
        l_wheel_in_world = T_world_car*left_wheel_pos;
        // std::cout << r_wheel_in_world << std::endl;
        // std::cout << "----------" << std::endl;
        

        x_frame_car_tip_x = x_frame_car(0,0);
        x_frame_car_tip_y = x_frame_car(1,0);
        y_frame_car_tip_x = y_frame_car(0,0);
        y_frame_car_tip_y = y_frame_car(1,0);

        std::vector<float> x_axis_x = {x_car_pos[0], x_frame_car_tip_x};
        std::vector<float> x_axis_y = {y_car_pos[0], x_frame_car_tip_y};
        std::vector<float> y_axis_x = {x_car_pos[0], y_frame_car_tip_x};
        std::vector<float> y_axis_y = {y_car_pos[0], y_frame_car_tip_y};
        
        r_wheel_x = r_wheel_in_world(0,0);
        r_wheel_y = r_wheel_in_world(1,0);
        l_wheel_x = l_wheel_in_world(0,0);
        l_wheel_y = l_wheel_in_world(1,0);
        std::vector<float> r_wheel_in_world_x = {r_wheel_x};
        std::vector<float> r_wheel_in_world_y = {r_wheel_y};
        std::vector<float> l_wheel_in_world_x = {l_wheel_x};
        std::vector<float> l_wheel_in_world_y = {l_wheel_y};


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