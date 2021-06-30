#include "ICamera.h"

#include <utility>

ICamera::ICamera(int deviceId, std::string camType) : camType(std::move(camType)) {
    cam = cv::VideoCapture(0);

    if(!cam.isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
    }
}

ICamera::~ICamera() {
    if(IsCapturing())
        StopCapture();
    cam.release();
}

/**
 * Setup the cameras' parameters.
 */
void ICamera::Setup() {
    // prevent auto focus and set focus to maximum value
    DisableAutofocus();
    SetFocusToInfinity();

    this->isSetup = true;
}

cv::Mat ICamera::getFrame() {
    checkSetup();

    cv::Mat frame;
    cam >> frame;
    return frame;
}


void ICamera::DisableAutofocus() {
    std::cerr << "This methods needs to be overridden in child class" << std::endl;
}

void ICamera::SetFocusToInfinity() {
    std::cerr << "This methods needs to be overridden in child class" << std::endl;
}

void ICamera::StartCapture() {
    isCapturing = true;
    // TODO implement this
}

void ICamera::StopCapture() {
    isCapturing = false;
    // TODO implement this
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
