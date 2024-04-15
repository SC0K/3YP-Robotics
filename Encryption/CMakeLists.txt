cmake_minimum_required(VERSION 3.12) 

project(Blowfish) 


set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)


add_executable(Blowfish test.cpp blowfish.cpp fileManagement.cpp)