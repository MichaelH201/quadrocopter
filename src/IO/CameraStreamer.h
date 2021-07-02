#ifndef QUADROCOPTER_CAMERASTREAMER_H
#define QUADROCOPTER_CAMERASTREAMER_H

#include "thread"
#include "concurrent_queue.h"
#include "opencv2/opencv.hpp"
#include "ICamera.h"
#include "LogitechC920.h"

using namespace concurrency;

class CameraStreamer {
public:
    std::vector<ICamera*> cameras;
    std::vector<concurrent_queue<cv::Mat>*> frame_queues;
    std::vector<std::thread*> camera_threads;
    int camera_count;

    explicit CameraStreamer(std::vector<int> deviceIds);
    ~CameraStreamer();
};

#endif //QUADROCOPTER_CAMERASTREAMER_H
