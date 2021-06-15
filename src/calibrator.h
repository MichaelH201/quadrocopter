#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include "defines.h"

#include "scene/CameraIntrinsics.h"

using namespace cv;
using namespace std;

class CameraCalibrator {
public:
    CameraIntrinsics<double> intrinsics;
    const Size patternSize = Size(10,7);

    CameraCalibrator(CameraIntrinsics<double> intrinsics, Size patternSize);
    ~CameraCalibrator();
    bool calibrate();

private:
    bool detectCheckerboard(const Mat* frame, InputOutputArray corners);
    void calculateExtrinsics(const vector<vector<Point2f>>* imagePoints, OutputArray R, OutputArray t);
    void drawCheckerboardCorners(Mat img, InputArray corners, String* winName);
    static void openWindow(String* name);
};
