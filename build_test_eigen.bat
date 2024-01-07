@echo off
set SRC=src\test_eigen.cpp
set OUT=test_eigen

g++ %SRC% -o %OUT% -I C:\Users\jfernandez75\Documents\eigen-3.4.0 