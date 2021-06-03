#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>

using namespace cv;
using namespace std;

class Calibrator {
public:
    static void calibrateCamera(OutputArray R, OutputArray t);
    static void localizeCamera();
    static void drawPointsOnReference();

private:
    static Mat createImage(string fileName);
    static void openWindow(string name);
};
