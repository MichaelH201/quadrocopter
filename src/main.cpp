#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp> // drawing shapes
#include <opencv2/core/utils//logger.hpp>

#include "calibrator.h"
#include "scene/CameraIntrinsics.h"
#include "math/Vector.hh"

#include <iostream>

int main() {
    utils::logging::setLogLevel(utils::logging::LOG_LEVEL_WARNING);

    CameraIntrinsics<double> intrinsics;
    intrinsics.ImageSize = base::Vec2d(1920.0, 1080.0);
    // focal length 3,67mm / 0,00398mm/px = 922,11px
    intrinsics.FocalLength = 922.11f;
    intrinsics.PrincipalPoint = intrinsics.ImageSize * 0.5;
    intrinsics.RadialDistortion.clear();
    intrinsics.TangentialDistortion.clear();

    /*
    intrinsics.ImageSize = base::Vec2d(2160.0, 2160.0);
    intrinsics.FocalLength = 2500.0f;						 // focal length 35mm / 0,014mm/px = 2500px
    intrinsics.PrincipalPoint = intrinsics.ImageSize * 0.5;	 // -> image center
    intrinsics.RadialDistortion.clear();
    intrinsics.TangentialDistortion.clear();
    */

    double tileWidth = 0.027530875; // 118px * 0.0002333125 m/px;

    CameraCalibrator calibrator = CameraCalibrator(intrinsics, Size(10, 7), tileWidth);
    calibrator.calibrate(1);
    calibrator.calibrate(2);
    calibrator.calibrate(3);
    calibrator.calibrate(4);

    #ifdef DEBUG
    waitKey(0);
    #endif

    return 0;
}
