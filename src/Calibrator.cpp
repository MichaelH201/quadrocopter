#include "Calibrator.h"

#include <utility>

CameraCalibrator::CameraCalibrator(CameraStreamer& streamer, Size patternSize, double tileWidth)
: streamer(streamer), patternSize(move(patternSize)), tileWidth(tileWidth){}

CameraCalibrator::~CameraCalibrator() {
    cout << "Finished calibration" << endl;
}

bool CameraCalibrator::calibrate() {
    cout << "Start calibrating " << streamer.cameraCount << " Camera(s)." << endl;
    // camera -> all images -> image points
    vector<vector<vector<Point2f>>*> imgPoints = vector<vector<vector<Point2f>>*>();
    for(int i = 0; i < streamer.cameraCount; i++) {
        imgPoints.push_back(new vector<vector<Point2f>>());
    }
    vector<Mat>* frames = new vector<Mat>();

    cout << "Collecting reference images..." << endl;
    int imgCount = 0;
    int failCount = 0;
    float progress = 0;
    // collect images
    while(imgCount < maxCalibrationFrames) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        if(streamer.TryGetFrames(frames)) {
            bool foundAll = true;
            failCount = 0;
            vector<vector<Point2f>> allCorners = vector<vector<Point2f>>(streamer.cameraCount);

            #pragma omp parallel for default(none) shared(foundAll, frames, allCorners, failCount, progress, std::cout)
            for(int i = 0; i < streamer.cameraCount; i++) {
                Mat& frame = (*frames)[i];
                vector<Point2f> corners;
                if(detectCheckerboard(&frame, corners)) {
                    allCorners[i] = corners;
                } else {
                    foundAll = false;
                    failCount++;
                    std::cout << '\r' << progress << "%" << " - fail (" << failCount << ")" << std::flush;
                }
            }

            // we have a chessboard in all images!
            if(foundAll) {
                for(int i = 0; i < streamer.cameraCount; i++) {
                    imgPoints[i]->push_back(allCorners[i]);
                }
                imgCount++;

                progress = (float)imgCount*100 / (float)maxCalibrationFrames;
                std::cout << '\r' << progress << "%" << "               " << std::flush;

                // wait some time to reposition the chessboard (even on fail)
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
    }

    std::cout << std::endl;
    cout << "Calculating camera intrinsics..." << endl;

    for(int i = 0; i < streamer.cameraCount; i++) {
        applyIntrinsics(imgPoints[i], streamer.cameras[i]);
    }


    for(auto* p : imgPoints) {
        delete p;
    }

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

void CameraCalibrator::applyIntrinsics(const vector<vector<Point2f>>* imagePoints, ICamera* cam) const {
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
    calibrateCamera(objPoints, *imagePoints, Size(cam->intrinsics.ImageSize[0], cam->intrinsics.ImageSize[1]), camMat, distCoeffs, Rs, ts);

    cam->intrinsics.FocalLength = (camMat.at<double>(0,0) + camMat.at<double>(1,1)) / 2.0;
    cam->intrinsics.PrincipalPoint = base::Vec2d(camMat.at<double>(0,2), camMat.at<double>(1,2));

    double meanError = 0;
    for(int i = 0; i < objPoints.size(); i++) {
        vector<Point2f> imagePoints2;
        cv::projectPoints(objPoints[i], Rs, ts, camMat, distCoeffs, imagePoints2);
        double error = cv::norm(imagePoints[i], imagePoints2, cv::NORM_L2)/(double)imagePoints2.size();
        meanError += error;
    }

    cout << "re-projection error: " << meanError/(double)objPoints.size() << endl;
    //cout << "Intrinsics for camera " << cam->deviceId << ":" << endl;
    //cout << cam->intrinsics.toString() << endl;
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

    // TODO start here
    // https://stackoverflow.com/questions/22338728/calculate-object-points-in-opencv-cameracalibration-function
}


/* ******************
 *  PRIVATE & DEBUG
 ******************* */
void CameraCalibrator::drawCheckerboardCorners(Mat img, InputArray corners, String& winName) {
    string windowName = string("Image " + winName);
    drawChessboardCorners(img, patternSize, corners, true);
    openWindow(windowName, Size(1600, 900));
    imshow(windowName, img);
    waitKey(1);
}