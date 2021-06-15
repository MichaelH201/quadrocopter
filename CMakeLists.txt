cmake_minimum_required(VERSION 3.19)
project(Quadrocopter)

set(CMAKE_CXX_STANDARD 20)

set(OpenCV_DIR "C:/Program Files/OpenCV-4.5.2/build/x64/vc15/lib")
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)

set(Project_FILES
        src/main.cpp
        src/calibrator.cpp
        src/calibrator.h
        src/scene/CameraIntrinsics.cpp
        src/scene/CameraIntrinsics.h
        src/math/MSVC_Fixes.h
        src/defines.h)

add_executable(Quadrocopter ${Project_FILES})

set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs)
target_link_libraries(Quadrocopter ${OpenCV_LIBS})
if(OpenMP_CXX_FOUND)
    target_link_libraries(Quadrocopter OpenMP::OpenMP_CXX)
endif()