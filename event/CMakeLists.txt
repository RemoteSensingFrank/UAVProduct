cmake_minimum_required(VERSION 3.5)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -Wall -Wno-deprecated" )

find_path(LibEvent_INCLUDE_DIR event.h PATHS
        /usr/local/include/evnet2/
        )
set(LibEvent_LIB_PATHS /usr/local/lib)
find_library(LibEvent_LIB NAMES event PATHS ${LibEvent_LIB_PATHS})

if (LibEvent_LIB AND LibEvent_INCLUDE_DIR)
    set(LibEvent_FOUND TRUE)
    set(LibEvent_LIB ${LibEvent_LIB})
    set(EVENTLIST eventSender.h eventSender.cpp)
    ADD_LIBRARY(eventlib SHARED ${EVENTLIST})
    TARGET_LINK_LIBRARIES(eventlib ${LibEvent_LIB})
else ()
    set(LibEvent_FOUND FALSE)
endif ()


