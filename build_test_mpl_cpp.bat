@echo off
set SRC=src\test_mpl_cpp.cpp
set OUT=test_mpl

g++ %SRC% -o %OUT% -I C:\Python312\include -I c:\users\jfernandez75\appdata\roaming\python\python312\site-packages\numpy\core\include -L C:\Python312\libs -lpython312
