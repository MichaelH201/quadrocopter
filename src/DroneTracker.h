#ifndef QUADROCOPTER_DRONETRACKER_H
#define QUADROCOPTER_DRONETRACKER_H

#include "opencv2/opencv.hpp"
#include "opencv2/bgsegm.hpp"
#include <iostream>
#include "utils.h"
#include "limits"

class DroneTracker {
public:
    bool droneFound = false;

    DroneTracker();
    ~DroneTracker();
    void track(cv::Mat& frame, cv::Rect& rect);
    bool tryGetBoundingBox(cv::Rect2f& rect);

private:
    static const int MAX_BUFFER_LENGTH = 2;
    std::deque<cv::Mat> buffer;
    cv::Rect2f lastDetection = cv::Rect();
    int detectionCount = 0;
    static const int MAX_DETECTIONS = 10;
    cv::Rect currentBbox = cv::Rect();
    cv::Ptr<cv::BackgroundSubtractor> background;

    std::vector<cv::Rect2f> detectMovingObjects();
    void combineOverlappingBoundingBoxes(std::vector<cv::Rect2f>& bboxes);
    float minimalRectDistance(const cv::Rect2f& rect1, const cv::Rect2f& rect2);
    cv::Rect2f combineRects(const cv::Rect2f& rect1, const cv::Rect2f& rect2);
    bool tryDetectDrone(cv::Mat& frame, std::vector<cv::Rect2f>& bbs);
    void drawBoundingBoxes(cv::Mat& img, std::vector<cv::Rect2f>& bbs);
};


#endif //QUADROCOPTER_DRONETRACKER_H
