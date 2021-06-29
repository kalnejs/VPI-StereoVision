cmake_minimum_required(VERSION 3.10)

project(stereo_camera_chessboard)

add_executable(${PROJECT_NAME} stereo_camera_chessboard.cpp)

target_compile_definitions(${PROJECT_NAME} PRIVATE -DDEBUG)

target_link_libraries(${PROJECT_NAME} camera)
target_link_libraries(${PROJECT_NAME} opencv_core)
target_link_libraries(${PROJECT_NAME} opencv_highgui)
target_link_libraries(${PROJECT_NAME} opencv_videoio)
target_link_libraries(${PROJECT_NAME} opencv_imgcodecs)
target_link_libraries(${PROJECT_NAME} opencv_imgproc)
target_link_libraries(${PROJECT_NAME} opencv_calib3d)
target_link_libraries(${PROJECT_NAME} pthread)
