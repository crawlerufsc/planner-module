# require CMake 2.8 or greater
cmake_minimum_required(VERSION 3.5)

project(driveless-planner-module)
include(GNUInstallDirs)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED) 

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

find_package(PkgConfig REQUIRED)
find_package(LibDataChannel REQUIRED)

pkg_check_modules(deps REQUIRED IMPORTED_TARGET glib-2.0)
pkg_search_module(GLIB REQUIRED glib-2.0) 
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

pkg_check_modules(Mosquitto IMPORTED_TARGET libmosquitto REQUIRED)
pkg_check_modules(Mosquittopp IMPORTED_TARGET libmosquittopp REQUIRED)

find_library(WIRINGPI_LIBRARIES NAMES wiringPi)
find_path(WIRINGPI_INCLUDE_DIRS NAMES wiringPi.h)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(wiringPi DEFAULT_MSG WIRINGPI_LIBRARIES WIRINGPI_INCLUDE_DIRS)

include_directories(/usr/include/gstreamer-1.0)
include_directories(/usr/include/crawler)

# control lib
find_library(CONTROL_LIB REQUIRED
    NAMES crawler_control
    HINTS /usr/lib
)

# framework lib
find_library(FRAMEWORK_LIB REQUIRED
    NAMES crawler_fw
    HINTS /usr/lib
)

SET(TARGET_H
    control/execution_controller.h
    control/vehicle_controller.h
    control/master_control_api.h
    control/hardware_keepalive.h
    #communication/webrtc.h
    #communication/webrtc_api_stream.h
    utils/image_utils.h
    model/planning_waypoint.h
    model/global_definitions.h
)

add_executable(
    planner-bin 
    planner.cpp 
    #communication/webrtc_api_stream.cpp
    control/execution_controller.cpp
    control/master_control_api.cpp
    control/vehicle_controller.cpp
    control/hardware_keepalive.cpp
)

target_link_libraries(planner-bin PkgConfig::deps 
    ${OpenCV_LIBS} 
    ${GST_APP_LIBRARIES} 
    ${GST_VIDEO_LIBRARIES} 
    ${CONTROL_LIB}
    ${FRAMEWORK_LIB}
    LibDataChannel::LibDataChannel
    PkgConfig::Mosquitto
    PkgConfig::Mosquittopp
    ${WIRINGPI_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
)