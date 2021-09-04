#include <opencv2/core/core.hpp>
#include <opencv2/core/utils//logger.hpp>

#include "Calibrator.h"
#include "Triangulation.h"

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
    vector<int> deviceIds = {0, 1, 2};
    CameraStreamer streamer(deviceIds, true);

    // start calibration
    CameraCalibrator calibrator(streamer, patternSize, tileWidth);
    calibrator.maxCalibrationFrames = 5;
    calibrator.calibrate();

    //streamer.activateDroneTracking();

    Triangulation triag(streamer);

    while(true) {
        triag.triangulate();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
