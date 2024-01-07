#include <iostream>
#include <Eigen/Dense>
#include <typeinfo>

 
using Eigen::MatrixXd;
using Eigen::VectorXd;
// using Eigen::Matrix;
/*
When compiling:
g++ -I C:\Users\jfernandez75\Documents\eigen-3.4.0 test_eigen.cpp -o test_eigen
*/
int main()
{
    float number;
    Eigen::MatrixXd T_world_car(4,4);
    T_world_car << 1, 0, 0, 0,
                   0, 1, 0, 0,
                   0, 0, 1, 0,
                   0, 0, 0, 1;
    Eigen::VectorXd v(6);
    v << 1, 2, 3, 4, 5, 6;
    Eigen::MatrixXd x_unit(4,1);
    x_unit << 2, 0, 0, 1;
    Eigen::MatrixXd x_frame_car(4,1);
    x_frame_car = T_world_car*x_unit;

    std::cout << x_frame_car << std::endl;
    std::cout << x_frame_car(0) << std::endl;
    std::cout << x_frame_car(3) << std::endl;

    number = x_frame_car(3);



    // std::cout << "0.1 * v =\n" << 0.1 * v << std::endl;
    // std::cout << "Doing v *= 2;" << std::endl;
    // v *= 2;
    // std::cout << "Now v =\n" << v << std::endl;

    // Eigen::VectorXd x(5);
    // x << 1, 2, 3, 4, 5;
    // Eigen::VectorXd y(5);
    // y << 5,4,3,2,1;

    // Eigen::MatrixXd C(x.rows(), x.cols()+y.cols());
    // C << x,y;

    // std::cout << "Result Vector:" <<std::endl << C << std::endl;

}