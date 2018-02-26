 cmake_minimum_required(VERSION 3.5)
 project(UAVAlgorithm)

 find_package(OpenGL REQUIRED)
 find_package(GLUT REQUIRED)
 find_package(GLEW REQUIRED)
 find_package(GDAL REQUIRED)

 set(CMAKE_CXX_STANDARD 11)
 set(Headers UAVDistanceTransform.h)
 set(CPPS UAVDistanceTransform.cpp UAVDistanceTransformGL.cpp)

 set(SOURCE_FILES ${Headers} ${CPPS})
 add_definitions(-DANALYSIS)
 add_library(UAVAlgorithm SHARED ${SOURCE_FILES})