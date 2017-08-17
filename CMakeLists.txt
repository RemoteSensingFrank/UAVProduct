cmake_minimum_required(VERSION 3.5)
#SET(CMAKE_BUILD_TYPE "Debug")
#SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
project(UAVProduct)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD 11)

#gdal include and linrary files
set(GDAL_LIBRARY "/usr/local/lib/libgdal.so")
set(GDAL_INCLUDE "/usr/local/include/")
set(SIFT_LIBRARY "/usr/local/lib/libvlsift.a")
set(EXIF_LIBRARY "/usr/local/lib/libeasyexif.a")
set(GTest_LIBRARY "/usr/local/lib/libgtest.a")

#openMP
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#openMVG
find_package(OpenMVG REQUIRED)
find_package(LibKML REQUIRED)
find_package(OpenMVS REQUIRED)
find_package(GTest REQUIRED)
find_package(DevIL REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${OPENMVG_INCLUDE_DIRS})
include_directories(${GDAL_INCLUDE})
include_directories(${OpenMVS_INCLUDE_DIRS})
include_directories(${LibKML_INCLUDE_DIRS})
#include_directories(${IL_INCLUDE_DIRS})


ADD_SUBDIRECTORY(SiftGPU)
LINK_DIRECTORIES(SiftGPU)

set(SOURCE_FILES ${SIFTLIST} UAVMain.cpp UAVXYZToLatLonWGS84.cpp UAVXYZToLatLonWGS84.h UAVGeoProc.h UAVGeoProc.cpp UAVGeoMosaic.h UAVGeoMosaic.cpp UAVDataList.cpp UAVDataList.h UAVAuxiliary.h UAVAuxiliary.cpp UAVBundle.cpp UAVBundle.h UAVFeatureExtract.cpp UAVDMatch.h UAVDenseProcess.cpp UAVDenseProcess.h UAVDenseExtract.cpp UAVCommon.h UAVCommon.cpp UAVFeatureExtractGpu.cpp)

add_executable(UAVProcess ${SOURCE_FILES})

target_link_libraries(UAVProcess ${OPENMVG_LIBRARIES})
target_link_libraries(UAVProcess ${OpenMVS_LIBRARIES})
target_link_libraries(UAVProcess ${GTEST_LIBRARIES})
target_link_libraries(UAVProcess ${LibKML_LIBRARIES})
#target_link_libraries(UAVProcess ${IL_LIBRARIES})
#target_link_libraries(UAVProcess ${ILU_LIBRARIES})
#target_link_libraries(UAVProcess ${ILUT_LIBRARIES})

target_link_libraries(UAVProcess siftgpu)
target_link_libraries(UAVProcess ${GDAL_LIBRARY})
target_link_libraries(UAVProcess ${SIFT_LIBRARY})
target_link_libraries(UAVProcess ${EXIF_LIBRARY})
