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
    
    std::string* winName;
    for(int i = 0; i < streamer.cameraCount; i++) {
        winName = new std::string("Cam " + to_string(i));
        openWindow(winName);
    }

    while(waitKey(10) <= 0) {
        if(streamer.TryGetFrames(frames)) {
            for(int i = 0; i < frames->size(); i++) {
                Mat frame = (*frames)[i];
                vector<Point2f> corners;

                if(detectCheckerboard(&frame, corners)) {
                    //imgPoints->push_back(corners);
                    drawChessboardCorners(frame, patternSize, corners, true);
                }

                imshow("Cam " + to_string(i), (*frames)[i]);
            }
        }
    }

    delete winName;
/*
    int imgCount = 0;
    while(imgCount < 10) {
        if(streamer.TryGetFrames(frames)) {
            for(int i = 0; i < frames->size(); i++) {
                Mat frame = (*frames)[i];
                vector<Point2f> corners;
                if(detectCheckerboard(&frame, corners)) {
                    imgPoints->push_back(corners);
                }
            }
        }
    }
*/
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


/*
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
*/

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
    string* windowName = new string("Image " + *winName);
    drawChessboardCorners(img, patternSize, corners, true);
    openWindow(windowName, Size(1600, 900));
    imshow(*windowName, img);
}
