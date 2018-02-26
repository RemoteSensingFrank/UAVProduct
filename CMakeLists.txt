cmake_minimum_required(VERSION 3.2)

SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O1 -Wall")
SET(CMAKE_CXX_FLAGS "-std=c++0x -fopenmp -msse4.1 -mavx2")
project(UAVProduct)

set(CMAKE_CXX_STANDARD 11)


#add_subdirectory(GUITools)
add_subdirectory(UAVProduct)
