#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include "defines.h"
#include "utils.h"
#include "IO/CameraStreamer.h"

#include "scene/CameraIntrinsics.h"

using namespace cv;
using namespace std;

class CameraCalibrator {
public:
    CameraIntrinsics intrinsics;
    const Size patternSize;
    const double tileWidth;
    int maxCalibrationFrames = 10;

    CameraCalibrator(CameraStreamer& streamer, Size patternSize, double tileWidth);
    bool calibrate();

private:
    CameraStreamer& streamer;

    bool detectCheckerboard(const Mat* frame, InputOutputArray corners);
    void applyIntrinsics(const vector<vector<Point2f>>* imagePoints, ICamera* cam) const;
    bool calculateExtrinsicsOffset();
    void drawCheckerboardCorners(Mat img, InputArray corners, String& winName);
};
