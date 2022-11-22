# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

project(manual-control-test)

set(CMAKE_CXX_STANDARD 14)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(Mosquitto IMPORTED_TARGET libmosquitto REQUIRED)

# control lib
find_library(CONTROL_LIB REQUIRED
    NAMES crawler_control
    #HINTS "${CMAKE_PREFIX_PATH}../../../control/lib"
    
)
include_directories(../control/lib/include)

SET(TARGET_H
    ../../control/vehicle_controller.h
    ../../control/manual_control_api.h
)

add_executable(
    manual_ctrl_tst
    manual_control_api_test.cpp
    ../../control/vehicle_controller.cpp
)

# link my-recognition to jetson-inference library
target_link_libraries(manual_ctrl_tst PkgConfig::deps ${CONTROL_LIB} PkgConfig::Mosquitto)