# Dubin's Car

##### This repository defines a Dubin's Car and moves it through teleoperation or through a waypoint (controlled by the user). In this repository "main_teleop.cpp" allows for teleoperation of the Dubin's car and "main.cpp" uses a waypoint (controlled by the user) to command car movement. Both scripts visual the car movement in matplotlib through python bindings. Additionally, there are other scripts to test the ability to use matplotlib and Eigen (for matrix multiplication).  

## Teleoperated Dubin's Car
You can teleoperate the Dubin's Car through "main_teleop.cpp". The controls of this script can be seen below:
- "w" = speed up
- "s" = stop vehicle movement
- "x" = reverse
- "a" = turn left
- "d" = turn right
- "e" = stop turning
- "q" = quit program

Note that in this program, when turning, you are increasing the angular velocity of the vehicle and switching turning direction will not instantly stop you from turning (it will just decrease the rate of the turn). Therefore, if you want to completely stop turning, then use "e". 

To run the script, you have to include the header file "matplotlibcpp.h" for visualization. Additionally, to compile, you can run below command in windows terminal. 

`build_main_teleop.bat`

Note, that you will need to include your path to eigen (https://eigen.tuxfamily.org/dox/GettingStarted.html), Python, and your numpy package within python. Additionally, you will need to link your python version. See this video for more clear details (https://www.youtube.com/watch?v=43Wp_qxyXDM).

In Addition to leveraging matplotlib and eigen, we also use the dubins_car class (found in "dubins_car.cpp") to command vehicle movement. This script defines our vehicle movement (dubins_car::car_model) and gets the cartesian velocity from velocity and angle of the vehicle (dubins_car::get_cartesian_v). Note, that this model uses realistic vehicle dynamics and does not assume the desired velocity and twist is achieved instantaneously. Therefore, we use a low-level PID controller to drive the wheel velocities to achieve our desire velocity and twist commands (dubins_car::PID). 


## Waypoint Dubin's Car
You can control the Dubin's Car by controlling a waypoint in "main.cpp". To control the waypoint:
- "w" = move up
- "x" = move down
- "a" = move left
- "d" = move right
- "q" = quit program 

To run the script, you will have to follow the same steps as when using the teleoperated dubins car. You can run the below command in windows terminal once you have corrected the paths: 

`build_main.bat`

In addition to using all the same functionality as the teleoperated dubins car, we also use a highlevel PID controller to command desired velocity and twist commands of the vehicle. This script just uses the relative position of the waypoint to command the velocity and twist (angular velocity) needed to get to the position of the waypoint. 

