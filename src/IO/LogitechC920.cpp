#include "LogitechC920.h"

LogitechC920::LogitechC920(int deviceId) : ICamera(deviceId, "Logitech C920 HD") {}


LogitechC920::LogitechC920(int deviceId, const std::vector<int>& imageSize) : ICamera(deviceId, "Logitech C920 HD", imageSize) {}


void LogitechC920::DisableAutofocus() {
    cam->set(cv::CAP_PROP_AUTOFOCUS, 0);
    //cam->set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    //cam->set(cv::CAP_PROP_EXPOSURE, -1);
}

void LogitechC920::SetFocusToInfinity() {
    cam->set(cv::CAP_PROP_FOCUS, 255);
}

void LogitechC920::SetResolution(int width, int height) {
    cam->set(cv::CAP_PROP_FRAME_WIDTH, width);
    cam->set(cv::CAP_PROP_FRAME_HEIGHT, height);
}

