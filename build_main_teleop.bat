@echo off
set SRC=src\main_teleop.cpp
set OUT=main_teleop

g++ %SRC% src\dubins_car.cpp -o %OUT% -I C:\Users\jfernandez75\Documents\eigen-3.4.0 -I C:\Python312\include -I c:\users\jfernandez75\appdata\roaming\python\python312\site-packages\numpy\core\include -L C:\Python312\libs -lpython312
