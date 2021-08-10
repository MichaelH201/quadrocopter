#ifndef QUADROCOPTER_DRONETRACKER_H
#define QUADROCOPTER_DRONETRACKER_H

#include "opencv2/opencv.hpp"
#include "IO/CameraStreamer.h"
#include <iostream>
#include "utils.h"

class DroneTracker {
public:
    explicit DroneTracker(CameraStreamer& streamer);
    void track();

private:
    const int MAX_BUFFER_LENGTH = 3;
    CameraStreamer& streamer;
    std::deque<cv::Mat> buffer;
    cv::Mat backgroundModel;

    std::vector<cv::Rect2f> detectMovingObjects(cv::Mat& img);
    void createBackgroundModel();
};


#endif //QUADROCOPTER_DRONETRACKER_H
