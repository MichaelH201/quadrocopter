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

    // theoretical focal length 3,67mm / 0,00398mm/px = 922,11px

    /**********************/
    /** input parameters **/
    /**********************/
    double tileWidth = 0.0325; // from paper 32.5mm
    Size patternSize(7,5); // amount of inner tiles of the chessboard

    // set up the camera streamer
    vector<int> deviceIds = {0};
    CameraStreamer streamer(deviceIds, true);

    // start calibration
    // TODO implement calibration as module for cameras to execute them in their own thread
    CameraCalibrator calibrator(streamer, patternSize, tileWidth);
    calibrator.maxCalibrationFrames = 5;
    calibrator.calibrate();

    return 0;
}
