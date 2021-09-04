#include "Triangulation.h"

Triangulation::Triangulation(CameraStreamer &streamer) : streamer(streamer) {}


cv::Vec3f Triangulation::triangulate() {
    std::vector<cv::Mat> frames;
    streamer.GetFrames(frames);

    cv::Size patternSize(7,5);
    double tileWidth = 0.0325;
    //std::vector<cv::Point3f> axis {cv::Point3f(0, 0, 0), cv::Point3f(3 * tileWidth, 0, 0), cv::Point3f(0, 3 * tileWidth, 0), cv::Point3f(0, 0, -3 * tileWidth)};
    std::vector<cv::Point3f> origin{cv::Point3f(0, 0, 0)};

    cv::Mat gray, r, R, t;
    std::vector<cv::Point2f> corners, undistorted;
    std::vector<cv::Point3f> directions(streamer.cameraCount);
    for(int i = 0; i < streamer.cameraCount; i++) {
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
            d = streamer.cameras[i]->RotationMatrix.inv() * d;
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
    // 400x400 should be 2x2 meter => factor: 200
    float fac = 200.0f;
    cv::Point origin(200, 300);

    cv::line(camOverview, cv::Point(-6, 0) + origin, cv::Point(6, 0) + origin, cv::Scalar(255, 255, 255), 1);
    cv::line(camOverview, cv::Point(0, -6) + origin, cv::Point(0, 6) + origin, cv::Scalar(255, 255, 255), 1);
    cv::putText(camOverview, "origin", cv::Point(3, 15) + origin, cv::FONT_HERSHEY_DUPLEX, .5, cv::Scalar(255, 255, 255), 1);

    //ref cam
    cv::Mat refT = streamer.cameras[0]->TranslationVector;
    cv::Mat refR = streamer.cameras[0]->RotationMatrix;
    cv::Point refOrigin = cv::Point(refT.at<double>(0, 0) * fac, refT.at<double>(2, 0) * -fac) + origin;
    cv::circle(camOverview, refOrigin, 3, cv::Scalar(0, 0, 255), 3);
    cv::line(camOverview, refOrigin, refOrigin + cv::Point(directions[0].x * 400, directions[0].z * 400), cv::Scalar(0, 255, 0), 1);

    cv::Point cp;
    cv::Mat ct;
    for(int i = 1; i < streamer.cameraCount; i++) {
        ct = refT - refR * streamer.cameras[i]->TranslationVector;
        cp = cv::Point(ct.at<double>(0, 0) * fac, ct.at<double>(2, 0) * -fac) + origin;
        cv::circle(camOverview, cp, 3, cv::Scalar(255, 0, 0), 3);

        cv::line(camOverview, cp, cp + cv::Point(directions[i].x * 400, directions[i].z * 400), cv::Scalar(0, 255, 0), 1);
    }

    cv::imshow("Camera Overview", camOverview);
    cv::waitKey(1);
}
