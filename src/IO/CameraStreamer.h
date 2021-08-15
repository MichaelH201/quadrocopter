#ifndef QUADROCOPTER_CAMERASTREAMER_H
#define QUADROCOPTER_CAMERASTREAMER_H

#include "thread"
#include "shared_mutex"
#include "opencv2/opencv.hpp"
#include "ICamera.h"
#include "LogitechC920.h"

class CameraStreamer {
public:
    std::vector<ICamera*> cameras;
    std::vector<std::vector<cv::Mat>*> frame_queues;
    std::vector<std::thread*> camera_threads;
    int cameraCount;
    int* bufferIndex;

    explicit CameraStreamer(std::vector<int> deviceIds, bool debugMode = false);
    ~CameraStreamer();
    void GetFrames(std::vector<cv::Mat>& frames);
    cv::Mat GetFrame(int camIndex);
    void activateDroneTracking();

private:
    std::shared_mutex* mtx;
};

#endif //QUADROCOPTER_CAMERASTREAMER_H
