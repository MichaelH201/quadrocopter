#ifndef QUADROCOPTER_DRONETRACKER_H
#define QUADROCOPTER_DRONETRACKER_H

#include "opencv2/opencv.hpp"
#include <iostream>
#include "utils.h"

class DroneTracker {
public:
    void track(cv::Mat& frame);

private:
    static const int MAX_BUFFER_LENGTH = 3;
    std::deque<cv::Mat> buffer;

    std::vector<cv::Rect2f> detectMovingObjects(cv::Mat& img);
    void combineOverlappingBoundingBoxes(std::vector<cv::Rect2f>& bboxes);
    bool containsRect(const cv::Rect2f& rect1, const cv::Rect2f& rect2);
    cv::Rect2f combineRects(const cv::Rect2f& rect1, const cv::Rect2f& rect2);
};


#endif //QUADROCOPTER_DRONETRACKER_H
