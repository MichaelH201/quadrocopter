#ifndef QUADROCOPTER_TRIANGULATION_H
#define QUADROCOPTER_TRIANGULATION_H

#include "opencv2/opencv.hpp"
#include "IO/CameraStreamer.h"
#include "scene/Ray.h"

class Triangulation {
public:
    explicit Triangulation(CameraStreamer& streamer);
    cv::Vec3f triangulate();

private:
    CameraStreamer& streamer;
    cv::Mat calcLeastSquarePoint(std::vector<Rayd>& rays);
    cv::Mat lmOptimization(std::vector<Rayd>& rays, cv::Mat& currPos, std::vector<cv::Point2f> imagePoints);

    /// tests ///
    void draw(std::vector<Rayd>& rays, cv::Vec3f dronePosition);
    void drawCamera(cv::Mat& frame, cv::Mat& t, cv::Mat& R, Rayd& ray, cv::Point& origin, float fac, const cv::Scalar& color = cv::Scalar(255, 0, 0));
};


#endif //QUADROCOPTER_TRIANGULATION_H
