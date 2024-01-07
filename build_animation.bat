@echo off
set SRC=src\animation.cpp
set OUT=animation_test

g++ %SRC% -o %OUT% -I C:\Users\jfernandez75\Documents\eigen-3.4.0 -I C:\Python312\include -I c:\users\jfernandez75\appdata\roaming\python\python312\site-packages\numpy\core\include -L C:\Python312\libs -lpython312
