#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp> // drawing shapes

#include "calibrator.h"

#include <iostream>

using namespace cv;

int main() {

    Mat R, t;

    // compute intrinsics
    Calibrator::calibrateCamera(R, t);
    Calibrator::drawPointsOnReference();

    //compute extrinsics
    Calibrator::localizeCamera();

    return 0;
}
