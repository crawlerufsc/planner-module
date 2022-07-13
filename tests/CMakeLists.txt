# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

project(test-modules)

set(CMAKE_CXX_STANDARD 14)
include_directories(${PROJECT_SOURCE_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(deps REQUIRED IMPORTED_TARGET glib-2.0)
pkg_search_module(GLIB REQUIRED glib-2.0) 
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

include_directories(/usr/include/gstreamer-1.0)


SET(TARGET_H
    ../communication/stream_reader.h
)

add_executable(
    planner-tst
    planner_tests.cpp
    ../communication/stream_reader.cpp
    communication/test_stream_reader.cpp 
    
)

# link my-recognition to jetson-inference library
target_link_libraries(tst PkgConfig::deps  ${GST_APP_LIBRARIES} ${GST_VIDEO_LIBRARIES})