#include "..\include\dubins_car.h"
#include "..\include\cartesian_v.h"
#include <iostream>
#include <stdio.h>
#include <math.h> 



dubins_car::dubins_car(){
    int number = 0;
};

cartesian_v dubins_car::get_cartesian_v(float velocity, float theta){

    car_cartesian_v.vel_x = velocity*cos(theta);
    car_cartesian_v.vel_y = velocity*sin(theta);
    return car_cartesian_v;

};

void dubins_car::SayHello(){
    std::cout<<"Hello from the monster!"<<std::endl;
}