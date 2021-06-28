cmake_minimum_required(VERSION 3.10)

project(stereo_camera_capture)

add_executable(${PROJECT_NAME} stereo_camera_capture.cpp)

target_link_libraries(${PROJECT_NAME} camera)
target_link_libraries(${PROJECT_NAME} opencv_core)
target_link_libraries(${PROJECT_NAME} opencv_highgui)
target_link_libraries(${PROJECT_NAME} opencv_videoio)
target_link_libraries(${PROJECT_NAME} opencv_imgcodecs)
target_link_libraries(${PROJECT_NAME} pthread)
