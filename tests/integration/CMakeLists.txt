# require CMake 3.8 or greater
cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_STANDARD 14)

project(driveless-planner-module-tst)

find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(ompl REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})
include_directories(/usr/include/gstreamer-1.0)
include_directories(${OMPL_INCLUDE_DIRS})

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

pkg_check_modules(deps REQUIRED IMPORTED_TARGET glib-2.0)
pkg_search_module(GLIB REQUIRED glib-2.0) 
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)


SET(TARGET_H
    ../../model/stream_data.h
    ../../communication/stream_data.h
    ../../communication/dataset_stream_reader.h
    ../../control/execution_control.h
    ../../control/process_pipeline.h
    ../../utils/file_utils.h
    ../../planning/planner.h
    ../../planning/ompl_planner.h
)

add_executable(
    planner-tst 
    planner.cpp 
    ../../communication/dataset_stream_reader.cpp
    ../../planning/ompl_planner.cpp
)

# link my-recognition to jetson-inference library
target_link_libraries(planner-tst PkgConfig::deps ${OpenCV_LIBS} ${GST_APP_LIBRARIES} ${GST_VIDEO_LIBRARIES} ${OMPL_LIBRARIES})