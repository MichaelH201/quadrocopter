#include "calibrator.h"

#include <utility>

CameraCalibrator::CameraCalibrator(CameraIntrinsics<double> intrinsics, Size patternSize)
: intrinsics(move(intrinsics)), patternSize(move(patternSize)){}

CameraCalibrator::~CameraCalibrator() = default;

bool CameraCalibrator::calibrate() {
    vector<vector<Point2f>>* imgPoints = new vector<vector<Point2f>>();

    // TODO: this should later iterate through camera stream frames
    // =====================================================================
    std::string path = "src/resources/images/";
    for(int i = 0; i < 3; i++) {
        Mat img = imread(path + "ref1" + to_string(i) + ".jpg");
        vector<Point2f> corners;
        if(detectCheckerboard(&img, corners)) {
            imgPoints->push_back(corners);

            #ifdef DEBUG
            drawCheckerboardCorners(img, corners, new String("image ref1" + to_string(i)));
            #endif
        }
    }
    // =====================================================================

    Mat R, t;
    calculateExtrinsics(imgPoints, R, t);

    cout << R << endl;
    cout << t << endl;

    return true;
}

bool CameraCalibrator::detectCheckerboard(const Mat* frame, InputOutputArray corners) {
    Mat gray;
    cvtColor(*frame, gray, COLOR_BGR2GRAY);

    // Find the chess board corners
    if(findChessboardCorners(*frame, patternSize, corners)) {
        cornerSubPix(gray, corners, Size(11,11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));
        return true;
    }

    return false;
}

void CameraCalibrator::calculateExtrinsics(const vector<vector<Point2f>>* imagePoints, OutputArray R, OutputArray t) {
    double cameraM[3][3] = {{intrinsics.FocalLength, 0.0, intrinsics.PrincipalPoint[0]}, {0.0, intrinsics.FocalLength, intrinsics.PrincipalPoint[1]}, {0.0, 0.0, 1.0}};
    Mat cameraMatrix = Mat(3, 3, CV_64F, cameraM);

    Mat E, mask;
    const vector<vector<Point2f>>& imgPoints = *imagePoints;
    E = findEssentialMat(imgPoints[0], imgPoints[1], cameraMatrix, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, imgPoints[0], imgPoints[1], cameraMatrix, R, t, mask);

    // TODO start here
    // https://stackoverflow.com/questions/22338728/calculate-object-points-in-opencv-cameracalibration-function
}


/* ******************
 *  PRIVATE & DEBUG
 ******************* */
void CameraCalibrator::drawCheckerboardCorners(Mat img, InputArray corners, String* winName) {
    String* windowName = new String("Image " + *winName);
    drawChessboardCorners(img, patternSize, corners, true);
    openWindow(windowName);
    imshow(*windowName, img);
}

void CameraCalibrator::openWindow(String* name) {
    int width = 1000;
    int height = 1000;
    namedWindow(*name, WINDOW_KEEPRATIO);
    resizeWindow(*name, width, height);
}

