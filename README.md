# dubins car

##### This repository defines a Dubin's Car and moves it through teleoperation or through a waypoint (controlled by the user). In this repository "main_teleop.cpp" allows for teleoperation of the Dubin's car and "main.cpp" uses a waypoint (controlled by the user) to command car movement. Both scripts visual the car movement in matplotlib through python bindings. Additionally, there are other scripts to test the ability to use matplotlib and Eigen (for matrix multiplication).  

## Teleoperated Dubin's Car
##### You can teleoperate the Dubin's Car through "main_teleop.cpp". The controls of this script can be seen below:
##### - "w" = speed up
##### - "s" = stop vehicle movement
##### - "x" = reverse
##### - "a" = turn left
##### - "d" = turn right
##### - "e" = stop turning
##### - "q" = quit program

##### Note that in this program, when turning, you are increasing the angular velocity of the vehicle and switching turning direction will not instantly stop you from turning (it will just decrease the rate of the turn). Therefore, if you want to completely stop turning, then use "e". 

##### To run the script, you have to include the header file "matplotlibcpp.h" for visualization. Additionally, to compile, you can use the "build_main_teleop.bat" (windows) as template. Note, that you will need to include your path to eigen (https://eigen.tuxfamily.org/dox/GettingStarted.html), Python, and your numpy package within python. Additionally, you will need to link your python version. See this video for more clear details (https://www.youtube.com/watch?v=43Wp_qxyXDM).


