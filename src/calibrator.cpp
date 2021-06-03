#include "calibrator.h"

vector<Point2f> Points[] = {
    vector<Point2f> {
            Point(487, 1368),
            Point(1057, 2120),
            Point(1484, 1686),
            Point(1021, 1043),
            Point(1038, 1547),
            Point(23, 1220),
            Point(1071, 2553),
            Point(1736, 1764),
            Point(1011, 746)
    },
    vector<Point2f> {
            Point(654, 1324),
            Point(582, 2240),
            Point(1528, 2307),
            Point(1587, 1350),
            Point(1078, 1800),
            Point(389, 1021),
            Point(253, 2534),
            Point(1847, 2670),
            Point(1922, 1047)
    },
    vector<Point2f> {
            Point(1344, 1285),
            Point(963, 1783),
            Point(1158, 2118),
            Point(1631, 1539),
            Point(1256, 1677),
            Point(1398, 1050),
            Point(802, 1841),
            Point(1085, 2449),
            Point(1943, 1425)
    }
};

/*
 * computes the intrinsics of the camera model
 */
void Calibrator::calibrateCamera(OutputArray R, OutputArray t) {
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat E, mask;
    E = findEssentialMat(Points[0], Points[1], cameraMatrix, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, Points[0], Points[1], cameraMatrix, R, t, mask);
}

void Calibrator::localizeCamera() {
    // TODO: implement camera localization
}

void Calibrator::drawPointsOnReference() {
    for(int i = 1; i < 4; i++) {
        string fileName = "ref0" + to_string(i);
        string windowName = "Image " + fileName;

        Mat img = createImage(fileName);

        // draw points
        for(int k = 0; k < Points[i-1].size(); k++) {
            Point2f p = Points[i-1][k];
            Scalar color(0,0,255);

            circle(img, p, 10, color, FILLED);
        }

        openWindow(windowName);
        imshow(windowName, img);
    }

    int k = waitKey(0);
}



/* ******************
 *  PRIVATE
 ******************* */

Mat Calibrator::createImage(string fileName) {
    std::string path = "E:/Nextcloud/Master/Semester 4/Forschungsprojekt/Quadrocopter/src/resources/images/" + fileName + ".jpg";
    Mat img = imread(path, IMREAD_COLOR);

    if(img.empty()) {
        throw std::invalid_argument("Image could not be opened. Name: " + fileName);
    }

    return img;
}

void Calibrator::openWindow(string name) {
    int width = 576;
    int height = 1024;
    namedWindow(name, WINDOW_KEEPRATIO);
    resizeWindow(name, width, height);
}
