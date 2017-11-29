cmake_minimum_required(VERSION 3.2)
#SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O1 -Wall")
project(UAVProduct)

set(CMAKE_CXX_STANDARD 11)




#openMP
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


find_package(OpenMVG REQUIRED)
find_package(OpenMVS REQUIRED)
#find_package(LibKML REQUIRED)
find_package(GTest REQUIRED)
find_package(DevIL REQUIRED)
find_package(GDAL REQUIRED)
find_package(PythonLibs 2.7 REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${OPENMVG_INCLUDE_DIRS})
include_directories(${GDAL_INCLUDE_DIRS})
include_directories(${OpenMVS_INCLUDE_DIRS})
include_directories(${LIBKML_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_PATH})

find_library(SIFT_LIBRARY libvlsift ${OPENMVG_LIBRARIES})
find_library(EXIF_LIBRARY libeasyexif ${OPENMVG_LIBRARIES})

ADD_SUBDIRECTORY(SiftGPU)
LINK_DIRECTORIES(SiftGPU)

set(INTERFACE common_header.h common_header.cpp UAVInterface.h)
set(WORKFLOW  UAVPreProcess.h UAVPreProcess.cpp UAVBundler.h UAVBundler.cpp UAVDMatch.h UAVDenseProcess.h UAVDenseProcess.cpp)
set(GEOPROCESS UAVProcessGeometry.h UAVProcessGeometry.cpp UAVMapCalculate.cpp UAVGeoCorrection.cpp)
set(SOURCE_FILES ${SIFTLIST} ${INTERFACE} ${WORKFLOW} ${GEOPROCESS} UAVMain.cpp UAVProcessThreadPool.h)

add_executable(UAVProcess ${SOURCE_FILES})

target_link_libraries(UAVProcess ${OPENMVG_LIBRARIES})
target_link_libraries(UAVProcess ${OpenMVS_LIBRARIES})
target_link_libraries(UAVProcess ${GTEST_LIBRARIES})
#target_link_libraries(UAVProcess ${LIBKML_LIBRARIES})

target_link_libraries(UAVProcess ${IL_LIBRARIES})
target_link_libraries(UAVProcess ${ILU_LIBRARIES})
target_link_libraries(UAVProcess ${ILUT_LIBRARIES})

target_link_libraries(UAVProcess siftgpu)
target_link_libraries(UAVProcess ${GDAL_LIBRARIES})
target_link_libraries(UAVProcess ${SIFT_LIBRARY})
target_link_libraries(UAVProcess ${EXIF_LIBRARY})
target_link_libraries(UAVProcess ${PYTHON_LIBRARIES})
