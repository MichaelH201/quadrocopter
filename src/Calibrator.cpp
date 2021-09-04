#include "Calibrator.h"

#include <utility>

CameraCalibrator::CameraCalibrator(CameraStreamer& streamer, Size patternSize, double tileWidth)
: streamer(streamer), patternSize(move(patternSize)), tileWidth(tileWidth){}

bool CameraCalibrator::calibrate() {
    cout << "Start calibrating " << streamer.cameraCount << " Camera(s)." << endl;
    auto* imgPoints = new vector<vector<Point2f>>();

    for(int i = 0; i < streamer.cameraCount; i++) {
        cout << "Calibrating Camera " << i << endl;
        imgPoints->clear();

        Mat frame;
        int imgCount = 0;
        int failCount = 0;
        float progress = 0;

        // collect images
        while(imgCount < maxCalibrationFrames) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            frame = streamer.GetFrame(i);
            vector<Point2f> corners;

            if(detectCheckerboard(&frame, corners)) {
                imgPoints->push_back(corners);
                imgCount++;
                failCount = 0;

                progress = (float)imgCount*100 / (float)maxCalibrationFrames;
                std::cout << '\r' << progress << "%" << "               " << std::flush;

                // wait some time to reposition the chessboard (even on fail)
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            } else {
                failCount++;
                std::cout << '\r' << progress << "%" << " - fail (" << failCount << ")" << std::flush;
            }
        }

        std::cout << std::endl;
        applyIntrinsics(imgPoints, streamer.cameras[i]);

        std::cout << "> Camera Matrix: " << std::endl << streamer.cameras[i]->CameraMatrix << std::endl;
    }

    setReferenceSpace();
    bool ret = calculateExtrinsicsOffset();
    cout << "Finished calibration" << endl;

    delete imgPoints;
    return ret;
}

void CameraCalibrator::setReferenceSpace() {
    cout << "Setup reference space" << endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    vector<Point2f> corners;
    Mat frame;

    vector<Point3f> objP;
    for(int y = 0; y < patternSize.height; y++) {
        for(int x = 0; x < patternSize.width; x++) {
            objP.push_back(Point3f(x*tileWidth, y*tileWidth, 0));
        }
    }

    while(true) {
        frame = streamer.GetFrame(0);
        if(detectCheckerboard(&frame, corners)) {

            cv::Mat R, r, t, c, d;
            c = streamer.cameras[0]->CameraMatrix;
            d = streamer.cameras[0]->intrinsics.DistortionCoefficients;

            cv::solvePnPRansac(objP, corners, c, d, r, t);
            cv::Rodrigues(r, R);

            streamer.cameras[0]->setExtrinsics(R, t);

            std::cout << "> Rotation Matrix: " << std::endl << streamer.cameras[0]->RotationMatrix << std::endl;
            std::cout << "> Translation Vector: " << std::endl << streamer.cameras[0]->TranslationVector << std::endl;
            break;
        }
    }
}

bool CameraCalibrator::calculateExtrinsicsOffset() {
    vector<vector<Point2f>> imgPoints;

    for(int i = 1; i < streamer.cameraCount; i++) {
        cout << "Calc extrinsic offset from camera " << streamer.cameras[i]->deviceId << " to camera " << streamer.cameras[0]->deviceId << endl;
        imgPoints.clear();

        vector<Mat> frames;
        bool searching = true;
        int failCount = 0;

        // collect images
        while(searching) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            streamer.GetFrames(frames);
            vector<Point2f> crns1, crns2;

            if(detectCheckerboard(&frames[i], crns1) && detectCheckerboard(&frames[0], crns2)) {
                imgPoints.push_back(crns1);
                imgPoints.push_back(crns2);
                searching = false;
                std::cout << '\r' << "success" << "   " << std::flush;
            } else {
                failCount++;
                std::cout << '\r' << "retry (" << failCount << ")" << std::flush;
            }
        }

        std::cout << std::endl;

        vector<vector<Point3f>> objectPoints;
        vector<Point3f> objP;
        for(int y = 0; y < patternSize.height; y++) {
            for(int x = 0; x < patternSize.width; x++) {
                objP.push_back(Point3f(x*tileWidth, y*tileWidth, 0));
            }
        }
        objectPoints.push_back(objP);

        cv::Mat R, t, E, F, c1, d1, c2, d2;
        c1 = streamer.cameras[i]->CameraMatrix;
        d1 = streamer.cameras[i]->intrinsics.DistortionCoefficients;
        c2 = streamer.cameras[0]->CameraMatrix;
        d2 = streamer.cameras[0]->intrinsics.DistortionCoefficients;

        vector<vector<Point2f>> imgPoints1, imgPoints2;
        imgPoints1.push_back(imgPoints[0]);
        imgPoints2.push_back(imgPoints[1]);
        cv::stereoCalibrate(objectPoints, imgPoints1, imgPoints2, c1, d1, c2, d2, Size(streamer.cameras[0]->intrinsics.ImageSize[0], streamer.cameras[0]->intrinsics.ImageSize[1]), R, t, E, F);
        /*
        cv::Mat r1, t1, r2, t2, c1, c2, d1, d2;
        c1 = streamer.cameras[i]->CameraMatrix;
        d1 = streamer.cameras[i]->intrinsics.DistortionCoefficients;
        c2 = streamer.cameras[0]->CameraMatrix;
        d2 = streamer.cameras[0]->intrinsics.DistortionCoefficients;


        cv::solvePnPRansac(objP, imgPoints[0], c1, d1, r1, t1);
        cv::solvePnPRansac(objP, imgPoints[1], c2, d2 ,r2, t2);

        cv::Mat R1, R2, R, t;
        cv::Rodrigues(r1, R1);
        cv::Rodrigues(r2, R2);
        R = R1.t() * R2;
        t = R1.t() * (t2 - t1);
        */

        streamer.cameras[i]->setExtrinsics(R, t);
        std::cout << "> Rotation Matrix: " << std::endl << streamer.cameras[i]->RotationMatrix << std::endl;
        std::cout << "> Translation Vector: " << std::endl << streamer.cameras[i]->TranslationVector << std::endl;
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

    Mat camMat, dist;
    std::vector<cv::Mat> Rs, ts;
    double err = calibrateCamera(objPoints, *imagePoints, Size(cam->intrinsics.ImageSize[0], cam->intrinsics.ImageSize[1]), camMat, dist, Rs, ts);

    cam->setCameraMatrix(camMat);
    cam->intrinsics.DistortionCoefficients = dist;
    cout << "re-projection error: " << err << endl;
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