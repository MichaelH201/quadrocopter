#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include "defines.h"
#include "utils.h"
#include "IO/LogitechC920.h"

#include "scene/CameraIntrinsics.h"

using namespace cv;
using namespace std;

class CameraCalibrator {
public:
    CameraIntrinsics<double> intrinsics;
    const Size patternSize = Size(10,7);
    const double tileWidth = 0.0002333125f;

    CameraCalibrator(vector<ICamera*>& cameras, Size patternSize, double tileWidth);
    ~CameraCalibrator();
    bool calibrate();

private:
    vector<ICamera*> _cameras;
    cv::Mat* _cameraBuffer[];

    bool detectCheckerboard(const Mat* frame, InputOutputArray corners);
    void calculateExtrinsics(const vector<vector<Point2f>>* imagePoints, OutputArray R, OutputArray t);
    void drawCheckerboardCorners(Mat img, InputArray corners, String* winName);
};
