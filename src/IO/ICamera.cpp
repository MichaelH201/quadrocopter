#include "ICamera.h"
#include "../utils.h"

#include <utility>

ICamera::ICamera(int deviceId, std::string type) : ICamera(deviceId, std::move(type), base::Vec2d(1280, 720)) {}

ICamera::ICamera(int deviceId, std::string type, const base::Vec2d& imageSize) : deviceId(deviceId), camType(std::move(type)) {
    cam = new cv::VideoCapture(deviceId);

    if(!cam->isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
    }

    intrinsics.ImageSize = imageSize;
}

ICamera::~ICamera() {
    if(IsCapturing())
        StopCapture();

    cam->release();
}

/**
 * Setup the cameras' parameters.
 */
void ICamera::Setup() {
    // prevent auto focus and set focus to maximum value
    DisableAutofocus();
    SetFocusToInfinity();
    SetResolution(intrinsics.ImageSize[0],intrinsics.ImageSize[1]);

    intrinsics.RadialDistortion.clear();
    intrinsics.TangentialDistortion.clear();

    this->isSetup = true;
    //std::cout << "Camera Setup: " << camType << " (DeviceId: " << deviceId << ")" << std::endl;
}

void ICamera::StartCapture(const int* bufferIndex, std::vector<cv::Mat>* multiBuffer, std::shared_mutex* mtx) {
    checkSetup();
    isCapturing = true;
    lastBufferIndex = *bufferIndex;

    if(displayFrames) {
        openWindow("cam" + std::to_string(deviceId));
    }

    while(IsCapturing()) {
        cv::Mat frame;
        (*cam) >> frame;

        // ====== thread safe ======
        mtx->lock_shared();
        int bIndex = (*bufferIndex);

        if(lastBufferIndex != bIndex) {
            SetFrameAvailable(lastBufferIndex, false);
            lastBufferIndex = bIndex;
        }

        multiBuffer->at(bIndex) = frame;
        SetFrameAvailable(bIndex, true);

        mtx->unlock_shared();
        // ====== thread safe end ======

        // for debug reasons
        if(displayFrames) {
            imshow("cam" + std::to_string(deviceId), frame);
            cv::waitKey(1);
        }

        frame.release();

        // sleep for a short duration
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void ICamera::StopCapture() {
    isCapturing = false;

    if(displayFrames)
        closeWindow("cam" + std::to_string(deviceId));
}

bool ICamera::IsCapturing() {
    return isCapturing;
}

bool ICamera::IsFrameAvailable(int index) {
    if(index == 0)
        return frameAvailableB1;
    else
        return frameAvailableB2;
}

void ICamera::SetFrameAvailable(int index, bool value) {
    if(index == 0)
        frameAvailableB1 = value;
    else
        frameAvailableB2 = value;
}

void ICamera::checkSetup() {
    if(!isSetup) {
        std::cerr << camType << " is not setup. Call Setup() before using the camera." << std::endl;
        throw std::bad_function_call();
    }
}

// ===================================== //
// virtual methods
// ===================================== //

void ICamera::SetResolution(int x, int y) {
    std::cerr << "The method SetResolution(int, int) needs to be implemented in child class" << std::endl;
}

void ICamera::DisableAutofocus() {
    std::cerr << "The method DisableAutofocus() needs to be implemented in child class" << std::endl;
}

void ICamera::SetFocusToInfinity() {
    std::cerr << "The method SetFocusToInfinity() needs to be implemented in child class" << std::endl;
}

