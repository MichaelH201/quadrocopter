#include "calibrator.h"

#include <utility>

CameraCalibrator::CameraCalibrator(CameraIntrinsics<double> intrinsics, Size patternSize, double tileWidth)
: intrinsics(move(intrinsics)), patternSize(move(patternSize)), tileWidth(tileWidth){}

CameraCalibrator::~CameraCalibrator() = default;

bool CameraCalibrator::calibrate(int cameraId) {
    vector<vector<Point2f>>* imgPoints = new vector<vector<Point2f>>();

    // TODO: this should later iterate through camera stream frames
    // =====================================================================
    std::string path = "src/resources/images/cam" + to_string(cameraId) + "/";
    for(int i = 0; i < 6; i++) {
        std::string imageSrc = path + "c" + to_string(cameraId) + "_rp" + to_string(i) + ".jpg";
        Mat img = imread(imageSrc);
        vector<Point2f> corners;
        if(detectCheckerboard(&img, corners)) {
            imgPoints->push_back(corners);

            #ifdef DEBUG
            drawCheckerboardCorners(img, corners, new String("Camera " + to_string(cameraId) + ", Pic " + to_string(i)));
            #endif
        }
    }
    // =====================================================================

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
    vector<vector<Point3f>> objPoints;

    E = findEssentialMat(imgPoints[0], imgPoints[1], cameraMatrix, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, imgPoints[0], imgPoints[1], cameraMatrix, R, t, mask);

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
    calibrateCamera(objPoints, *imagePoints, Size(intrinsics.ImageSize[0], intrinsics.ImageSize[1]), camMat, distCoeffs, Rs, ts);

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
    String* windowName = new String("Image " + *winName);
    drawChessboardCorners(img, patternSize, corners, true);
    openWindow(windowName);
    imshow(*windowName, img);
}

void CameraCalibrator::openWindow(String* name) {
    int width = 1600;
    int height = 900;
    namedWindow(*name, WINDOW_KEEPRATIO);
    resizeWindow(*name, width, height);
}

