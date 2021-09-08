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
    void drawCamera(cv::Mat& frame, cv::Mat& t, cv::Mat& R, cv::Point3f& rayDir, cv::Point& origin, float fac, const cv::Scalar& color = cv::Scalar(255, 0, 0));
};


#endif //QUADROCOPTER_TRIANGULATION_H
