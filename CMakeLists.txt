cmake_minimum_required(VERSION 3.10)

set(DEBUG)

project(camera_test)

find_package(vpi REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

add_executable(${PROJECT_NAME} camera/Camera.cpp tests/camera_test.cpp)

target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG)

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} vpi opencv_core opencv_highgui opencv_videoio)
target_link_libraries(${PROJECT_NAME} opencv_imgcodecs)
target_link_libraries(${PROJECT_NAME} pthread)
