#ifndef QUADROCOPTER_UTILS_H
#define QUADROCOPTER_UTILS_H

#include "opencv2/opencv.hpp"

static void openWindow(std::string* name, cv::WindowFlags flags = cv::WINDOW_AUTOSIZE) {
    cv::namedWindow(*name, flags);
}

static void openWindow(std::string* name, const cv::Size& winSize) {
    cv::namedWindow(*name, cv::WINDOW_KEEPRATIO);
    cv::resizeWindow(*name, winSize.width, winSize.height);
}

#endif //QUADROCOPTER_UTILS_H
