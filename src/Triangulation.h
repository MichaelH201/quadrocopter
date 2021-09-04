#ifndef QUADROCOPTER_TRIANGULATION_H
#define QUADROCOPTER_TRIANGULATION_H

#include "opencv2/opencv.hpp"
#include "IO/CameraStreamer.h"

class Triangulation {
public:
    explicit Triangulation(CameraStreamer& streamer);
    cv::Vec3f triangulate();

private:
    CameraStreamer& streamer;

    /// tests ///
    void draw(std::vector<cv::Point3f> directions);
};


#endif //QUADROCOPTER_TRIANGULATION_H
