#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp> // drawing shapes
#include <opencv2/core/utils//logger.hpp>

#include "Calibrator.h"
#include "IO/CameraStreamer.h"
#include "utils.h"
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

    double tileWidth = 0.027530875; // 118px * 0.0002333125 m/px;

    // swapping buffer damit die Kamera das Bild nicht überschreibt
    // mutex to wait until the frame is written
    vector<int> deviceIds = {0, 1};
    CameraStreamer streamer(deviceIds);

    CameraCalibrator calibrator(streamer, Size(10, 7), tileWidth);
    calibrator.calibrate();

    return 0;
}
