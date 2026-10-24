@echo off
rem configure once (native, MinGW g++ + Ninja), then build incrementally
if not exist build-desktop\CMakeCache.txt (
  cmake -S . -B build-desktop -G Ninja -D CMAKE_BUILD_TYPE=Debug -D CMAKE_C_COMPILER=gcc -D CMAKE_CXX_COMPILER=g++
)
cmake --build build-desktop
