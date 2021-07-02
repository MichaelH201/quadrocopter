#include "ICamera.h"
#include <utility>

ICamera::ICamera(int deviceId, std::string camType) : deviceId(deviceId), camType(std::move(camType)) {
    cam = new cv::VideoCapture(deviceId);

    if(!cam->isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
    }
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

    this->isSetup = true;
    std::cout << "Camera Setup: " << camType << " (DeviceId: " << deviceId << ")" << std::endl;
}

void ICamera::DisableAutofocus() {
    std::cerr << "The method DisableAutofocus() needs to be implemented in child class" << std::endl;
}

void ICamera::SetFocusToInfinity() {
    std::cerr << "The method SetFocusToInfinity() needs to be implemented in child class" << std::endl;
}

void ICamera::StartCapture(concurrent_queue<cv::Mat>* queue) {
    isCapturing = true;

    while(IsCapturing()) {
        cv::Mat frame;
        (*cam) >> frame;
        queue->push(frame);
        frame.release();

        if(queue->unsafe_size() > 100) {
            std::cerr << "Camera buffer is running full -> Size: " << queue->unsafe_size() << " (" << camType << ", Id: " << deviceId << ")" << std::endl;
        }
    }
}

void ICamera::StopCapture() {
    isCapturing = false;
}

bool ICamera::IsCapturing() {
    return isCapturing;
}

void ICamera::checkSetup() {
    if(!isSetup) {
        std::cerr << camType << " is not setup. Call Setup() before using the camera." << std::endl;
        throw std::bad_function_call();
    }
}
