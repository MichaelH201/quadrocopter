#include "Calibrator.h"

#include <utility>

CameraCalibrator::CameraCalibrator(CameraStreamer& streamer, Size patternSize, double tileWidth)
: streamer(streamer), patternSize(move(patternSize)), tileWidth(tileWidth){
    cout << "Start calibrating " << streamer.cameraCount << " Camera(s)." << endl;

}

CameraCalibrator::~CameraCalibrator() {
    cout << "Finished calibration" << endl;
}

bool CameraCalibrator::calibrate() {
    vector<vector<Point2f>>* imgPoints = new vector<vector<Point2f>>();
    vector<Mat>* frames = new vector<Mat>();

    int imgCount = 0;
    int failCount = 0;
    float progress = 0;
    // collect images
    while(imgCount < maxCalibrationFrames) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if(streamer.TryGetFrames(frames)) {
            // detect chessboard on all frames (supposed to work on 1 cam atm)
            for(auto frame : *frames) {
                vector<Point2f> corners;
                if(detectCheckerboard(&frame, corners)) {
                    imgPoints->push_back(corners);
                    imgCount++;

                    failCount = 0;
                    progress = (float)imgCount*100 / (float)maxCalibrationFrames;
                    std::cout << '\r' << progress << "%" << "               " << std::flush;

                    // wait some time to reposition the chessboard
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                } else {
                    failCount++;
                    std::cout << '\r' << progress << "%" << " - try fail (" << failCount << ")" << std::flush;
                }
            }
        }
    }

    std::cout << std::endl;

    Mat R, t;
    calculateExtrinsics(imgPoints, R, t);

    //cout << R << endl;
    //cout << t << endl;


    delete imgPoints;
    return true;
}


bool CameraCalibrator::detectCheckerboard(const Mat* frame, InputOutputArray corners) {
    Mat gray;
    cvtColor(*frame, gray, COLOR_BGR2GRAY);

    // Find the chess board corners
    if(findChessboardCorners(gray, patternSize, corners)) {
        cornerSubPix(gray, corners, Size(11,11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));
        return true;
    }

    return false;
}

void CameraCalibrator::calculateExtrinsics(const vector<vector<Point2f>>* imagePoints, OutputArray R, OutputArray t) {
 /*
    double cameraM[3][3] = {{intrinsics.FocalLength, 0.0, intrinsics.PrincipalPoint[0]}, {0.0, intrinsics.FocalLength, intrinsics.PrincipalPoint[1]}, {0.0, 0.0, 1.0}};
    Mat cameraMatrix = Mat(3, 3, CV_64F, cameraM);

    Mat E, mask;

    E = findEssentialMat(imgPoints[0], imgPoints[1], cameraMatrix, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, imgPoints[0], imgPoints[1], cameraMatrix, R, t, mask);
*/
    const vector<vector<Point2f>>& imgPoints = *imagePoints;
    vector<vector<Point3f>> objPoints;

    vector<Point3f> objP;
    for(int y = 0; y < patternSize.height; y++) {
        for(int x = 0; x < patternSize.width; x++) {
            objP.push_back(Point3f(x*tileWidth, y*tileWidth, 0));
        }
    }
    for(int i = 0; i < imagePoints->size(); i++) {
        objPoints.push_back(objP);
    }

    Mat camMat, distCoeffs, Rs, ts;
    calibrateCamera(objPoints, *imagePoints, Size(1280, 720), camMat, distCoeffs, Rs, ts);

    cout << "Camera Matrix" << endl;
    cout << camMat << endl;
    cout << endl;

    // TODO start here
    // https://stackoverflow.com/questions/22338728/calculate-object-points-in-opencv-cameracalibration-function
}


/* ******************
 *  PRIVATE & DEBUG
 ******************* */
void CameraCalibrator::drawCheckerboardCorners(Mat img, InputArray corners, String* winName) {
    string* windowName = new string("Image " + *winName);
    drawChessboardCorners(img, patternSize, corners, true);
    openWindow(windowName, Size(1600, 900));
    imshow(*windowName, img);
    waitKey(1);
}
