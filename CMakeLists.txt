cmake_minimum_required(VERSION 3.10)

project(stereo_vision)

find_package(vpi REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

set(DEBUG 1)

if(DEBUG)
  set(OUTPUT_DIR Debug)
else()
  set(OUTPUT_DIR Release)
endif()

include_directories(include)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${OUTPUT_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/${OUTPUT_DIR})

add_library(camera SHARED camera/Camera.cpp)

add_subdirectory(tests/single_camera_capture)
add_subdirectory(tests/stereo_camera_capture)
add_subdirectory(calibration)
