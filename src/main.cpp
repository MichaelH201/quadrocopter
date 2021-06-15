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
    intrinsics.ImageSize = base::Vec2d(2160.0, 2160.0);
    intrinsics.FocalLength = 0.025f;						 // focal length 35mm / 1400mm/px = 0.025px
    intrinsics.PrincipalPoint = intrinsics.ImageSize * 0.5;	 // -> image center
    intrinsics.RadialDistortion.clear();
    intrinsics.TangentialDistortion.clear();

    CameraCalibrator calibrator = CameraCalibrator(intrinsics, Size(10, 7));
    calibrator.calibrate();

    #ifdef DEBUG
    waitKey(0);
    #endif

    return 0;
}
