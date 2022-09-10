# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

project(driveless-planner-module)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

set(CMAKE_CXX_STANDARD 14)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(deps REQUIRED IMPORTED_TARGET glib-2.0)
pkg_search_module(GLIB REQUIRED glib-2.0) 
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

include_directories(/usr/include/gstreamer-1.0)


SET(TARGET_H
    communication/network_stream_reader.h
    communication/stream_data.h
    control/execution_control.h
    control/process_pipeline.h
)

add_executable(
    planner-bin 
    planner.cpp 
    communication/network_stream_reader.cpp
)

# link my-recognition to jetson-inference library
target_link_libraries(planner-bin PkgConfig::deps ${OpenCV_LIBS} ${GST_APP_LIBRARIES} ${GST_VIDEO_LIBRARIES})