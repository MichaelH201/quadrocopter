#include "ICamera.h"

ICamera::ICamera(int deviceId, std::string type) : ICamera(deviceId, move(type), base::Vec2d(1280.0, 720.0)) {}

ICamera::ICamera(int deviceId, std::string type, base::Vec2d imageSize) : deviceId(deviceId), camType(std::move(camType)) {
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
    SetResolution((int)intrinsics.ImageSize[0],(int)intrinsics.ImageSize[1]);

    intrinsics.RadialDistortion.clear();
    intrinsics.TangentialDistortion.clear();

    this->isSetup = true;
    std::cout << "Camera Setup: " << camType << " (DeviceId: " << deviceId << ")" << std::endl;
}

void ICamera::StartCapture(const int* bufferIndex, std::vector<cv::Mat>* multiBuffer, std::shared_mutex* mtx) {
    checkSetup();
    isCapturing = true;

    while(IsCapturing()) {

        cv::Mat frame;
        (*cam) >> frame;

        mtx->lock_shared();
        int bIndex = (*bufferIndex);
        multiBuffer->at(bIndex) = frame;
        SetFrameAvailable(bIndex, true);
        mtx->unlock_shared();

        frame.release();

        // sleep for a short duration
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void ICamera::StopCapture() {
    isCapturing = false;
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

