cmake_minimum_required(VERSION 3.19)
project(Quadrocopter)

set(CMAKE_CXX_STANDARD 20)

set(OpenCV_DIR "C:/Program Files/OpenCV-4.5.3/install/opencv/x64/vc16/lib")
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

set(Project_FILES
        src/main.cpp
        src/Calibrator.cpp
        src/Calibrator.h
        src/scene/CameraIntrinsics.h
        src/math/MSVC_Fixes.h
        src/defines.h
        src/IO/LogitechC920.cpp
        src/IO/LogitechC920.h
        src/utils.h src/IO/ICamera.h src/IO/ICamera.cpp src/IO/CameraStreamer.cpp src/IO/CameraStreamer.h src/DroneTracker.cpp src/DroneTracker.h)

add_executable(Quadrocopter ${Project_FILES})

set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs)
target_link_libraries(Quadrocopter ${OpenCV_LIBS})
if(OpenMP_CXX_FOUND)
    target_link_libraries(Quadrocopter OpenMP::OpenMP_CXX)
endif()