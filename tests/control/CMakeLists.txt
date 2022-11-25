# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

project(manual-control-test)

set(CMAKE_CXX_STANDARD 14)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED) 
pkg_check_modules(Mosquitto IMPORTED_TARGET libmosquitto REQUIRED)
pkg_check_modules(Mosquittopp IMPORTED_TARGET libmosquittopp REQUIRED)
pkg_check_modules(deps REQUIRED IMPORTED_TARGET glib-2.0)
pkg_search_module(GLIB REQUIRED glib-2.0) 

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_path(WIRINGPI_INCLUDE_DIRS NAMES wiringPi.h)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(wiringPi DEFAULT_MSG WIRINGPI_LIBRARIES WIRINGPI_INCLUDE_DIRS)

# control lib
find_library(CONTROL_LIB REQUIRED
    NAMES crawler_control
    HINTS /usr/lib
)
include_directories(/usr/include/crawler)

SET(TARGET_H
    ../../control/vehicle_controller.h
    ../../control/manual_control_api.h
    ../../utils/file_utils.h
)

add_executable(
    manual_ctrl_tst
    manual_control_api_test.cpp
    ../../control/vehicle_controller.cpp
    ../../control/manual_control_api.cpp
)

add_executable(
    vehicle_control_tst
    vehicle_control_test.cpp
    ../../control/vehicle_controller.cpp
)

# link my-recognition to jetson-inference library
target_link_libraries(manual_ctrl_tst PkgConfig::deps ${CONTROL_LIB} PkgConfig::Mosquitto  PkgConfig::Mosquittopp ${WIRINGPI_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(vehicle_control_tst PkgConfig::deps ${CONTROL_LIB}  ${WIRINGPI_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})