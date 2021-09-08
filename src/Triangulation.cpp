#include "Triangulation.h"

Triangulation::Triangulation(CameraStreamer &streamer) : streamer(streamer) {}


cv::Vec3f Triangulation::triangulate() {
    std::vector<cv::Mat> frames;
    std::vector<cv::Rect> boundingBoxes;
    streamer.GetFrames(frames, boundingBoxes);

    cv::Size patternSize(7,5);
    double tileWidth = 0.0325;
    //std::vector<cv::Point3f> axis {cv::Point3f(0, 0, 0), cv::Point3f(3 * tileWidth, 0, 0), cv::Point3f(0, 3 * tileWidth, 0), cv::Point3f(0, 0, -3 * tileWidth)};
    std::vector<cv::Point3f> origin{cv::Point3f(0, 0, 0)};

    cv::Mat gray, r, R, t;
    std::vector<cv::Point2f> corners, undistorted;
    std::vector<cv::Point3f> directions(streamer.cameraCount);

    for(int i = 0; i < streamer.cameraCount; i++) {

        /*
        cv::Point2f trackingPoint = boundingBoxes[i].tl() + cv::Point(boundingBoxes[i].width / 2, boundingBoxes[i].height/2);
        cv::Mat d = cv::Mat(3, 1, CV_64F);
        d.at<double>(0,0) = trackingPoint.x;
        d.at<double>(1,0) = trackingPoint.y;
        d.at<double>(2,0) = 1.0f;
        d = streamer.cameras[i]->CameraMatrix.inv() * d;
        d = streamer.cameras[i]->RotationMatrix.inv() * d;
        cv::Point3f dir(d.at<double>(0,0), d.at<double>(1, 0), d.at<double>(2,0));
        dir *= 1/cv::norm(dir);

        directions[i] = dir;
        */

        cv::cvtColor(frames[i], gray, cv::COLOR_BGR2GRAY);

        if(cv::findChessboardCorners(gray, patternSize, corners)) {
            cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            cv::undistortPoints(corners, undistorted, streamer.cameras[i]->CameraMatrix, streamer.cameras[i]->intrinsics.DistortionCoefficients);
            cv::Point2f trackingPoint = corners[0];
            cv::Mat d = cv::Mat(3, 1, CV_64F);
            d.at<double>(0,0) = trackingPoint.x;
            d.at<double>(1,0) = trackingPoint.y;
            d.at<double>(2,0) = 1.0f;
            d = streamer.cameras[i]->CameraMatrix.inv() * d;
            d = streamer.cameras[i]->RotationMatrix * d;

            cv::Point3f dir(d.at<double>(0,0), d.at<double>(1, 0), d.at<double>(2,0));
            dir *= 1/cv::norm(dir);

            directions[i] = dir;
        }
    }

    draw(directions);

    return cv::Vec3f();
}

void Triangulation::draw(std::vector<cv::Point3f> directions) {
    cv::Mat camOverview = cv::Mat::zeros(400, 400, CV_8UC3);
    // 400x400 should be 2x2 meter => 1m => 200 units
    float fac = 200.0f;
    cv::Point origin(200, 200);
    cv::putText(camOverview, "RC", cv::Point(3, 15) + origin, cv::FONT_HERSHEY_DUPLEX, .5, cv::Scalar(0, 0, 255), 1);

    //ref cam
    cv::Mat refT = streamer.cameras[0]->TranslationVector;
    cv::Mat refR = streamer.cameras[0]->RotationMatrix;
    drawCamera(camOverview, refT, refR, directions[0], origin, fac, cv::Scalar(0, 0, 255));

    //cv::Point cp;
    cv::Mat ct, cR;
    for(int i = 1; i < streamer.cameraCount; i++) {
        drawCamera(camOverview, streamer.cameras[i]->TranslationVector, streamer.cameras[i]->RotationMatrix, directions[i], origin, fac);
    }

    cv::imshow("Camera Overview", camOverview);
    cv::waitKey(1);
}

void Triangulation::drawCamera(cv::Mat& frame, cv::Mat& t, cv::Mat& R, cv::Point3f& rayDir, cv::Point& origin, float fac, const cv::Scalar& color) {
    cv::Mat o = t;
    cv::Point2f drawOrigin = cv::Point((int)(o.at<double>(0, 0) * fac), (int)(o.at<double>(2, 0) * fac)) + origin;
    cv::circle(frame, drawOrigin, 1, color, 2);

    cv::Mat forward = cv::Mat::zeros(3, 1, CV_64FC1);
    forward.at<double>(2, 0) = 1;
    forward = R * forward;
    cv::line(frame, drawOrigin, drawOrigin + cv::Point2f(forward.at<double>(0, 0), forward.at<double>(2, 0)) * 30, cv::Scalar(0, 255, 255), 1);
    cv::line(frame, drawOrigin, drawOrigin + cv::Point2f(rayDir.x, rayDir.z) * 400, cv::Scalar(0, 255, 0), 1);

}
