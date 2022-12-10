# require CMake 2.8 or greater
cmake_minimum_required(VERSION 2.8)

project(unittest)

set(CMAKE_CXX_STANDARD 14)
include_directories(${PROJECT_SOURCE_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(deps REQUIRED IMPORTED_TARGET glib-2.0)
pkg_search_module(GLIB REQUIRED glib-2.0) 
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

# Locate GTest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/include/gstreamer-1.0)

# framework lib
find_library(FRAMEWORK_LIB REQUIRED
    NAMES crawler_fw
    HINTS /usr/lib
)
include_directories(/usr/include/crawler)

SET(TARGET_H
  ../communication/gstream_client.h
  # ../control/callback_process_pipeline.h
  # ../control/direct_process_pipeline.h
  # ../communication/dataset_stream_reader.h
  # ../utils/debug_utils.h
  # ../../utils/data_convert.h
  # ../../model/frame.h
  # ../../model/planning_waypoint.h
  # ../../planning/ompl_skeletonize.h

)



add_executable(
  unittest
  planner_tests.cpp
  communication/test_gstream_client.cpp
  ../communication/gstream_client.cpp
  # control/callback_process_pipeline_test.cpp   
  # communication/test_dataset_stream_reader.cpp
  # ../communication/dataset_stream_reader.cpp
  # utils/data_convert_test.cpp
  # model/frame_test.cpp
  # model/planning_waypoint_test.cpp
  # planning/ompl/ompl_skeletonize_test.cpp
)

if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(unittest PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(unittest "${CMAKE_THREAD_LIBS_INIT}")
endif()

# link my-recognition to jetson-inference library
target_link_libraries(unittest PkgConfig::deps 
  ${GTEST_LIBRARIES} 
  ${GST_APP_LIBRARIES} 
  ${GST_VIDEO_LIBRARIES} 
  ${OpenCV_LIBS}
  ${CONTROL_LIB}
)