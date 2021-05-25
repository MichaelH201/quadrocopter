cmake_minimum_required(VERSION 3.19)
project(Quadrocopter)

set(CMAKE_CXX_STANDARD 20)

set(OpenCV_DIR "C:/Program Files/OpenCV-4.5.2/build/x64/vc15/lib")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Quadrocopter src/main.cpp)

set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs)

target_link_libraries(Quadrocopter ${OpenCV_LIBS})