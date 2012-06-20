@echo off

REM --------------------------
mkdir build
cd build
cmake -G "MinGW Makefiles" .. 
make 
cd ..
REM --------------------------
