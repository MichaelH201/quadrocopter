#include "Triangulation.h"

Triangulation::Triangulation(CameraStreamer &streamer) : streamer(streamer) {}


cv::Vec3f Triangulation::triangulate() {
    std::vector<cv::Mat> frames;
    std::vector<cv::Rect> boundingBoxes;
    streamer.GetFrames(frames, boundingBoxes);

    cv::Size patternSize(7,5);

    cv::Mat gray, r, R, t;
    std::vector<cv::Point2f> corners, undistorted;
    std::vector<Rayd> rays(streamer.cameraCount);

    //tmp
    std::vector<cv::Point2f> features(streamer.cameraCount);


    for(int i = 0; i < streamer.cameraCount; i++) {

        cv::Point2f trackingPoint = boundingBoxes[i].tl() + cv::Point(boundingBoxes[i].width / 2, boundingBoxes[i].height/2);
        //cv::Point2f trackingPoint = corners[0];
        features[i] = trackingPoint;

        Rayd ray = Rayd();
        ray.Origin = streamer.cameras[i]->TranslationVector;

        cv::Mat d = cv::Mat(3, 1, CV_64F);
        d.at<double>(0,0) = trackingPoint.x;
        d.at<double>(1,0) = trackingPoint.y;
        d.at<double>(2,0) = 1.0f;
        d = streamer.cameras[i]->CameraMatrix.inv() * d;
        d = streamer.cameras[i]->RotationMatrix * d;
        d *= 1/cv::norm(d);

        ray.Direction = d;
        rays[i] = ray;

        /*
        cv::cvtColor(frames[i], gray, cv::COLOR_BGR2GRAY);
        if(cv::findChessboardCorners(gray, patternSize, corners)) {
            cornerSubPix(gray, corners, cv::Size(11,11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            cv::undistortPoints(corners, undistorted, streamer.cameras[i]->CameraMatrix, streamer.cameras[i]->intrinsics.DistortionCoefficients);
        }*/
    }

    cv::Mat dronePosition = calcLeastSquarePoint(rays);
    //// drawing of reprojection /////
    for(int i = 0; i < streamer.cameraCount; i++) {
        cv::Mat frame = frames[i];
        cv::Mat cameraSpacePos = streamer.cameras[i]->RotationMatrix.inv() * (dronePosition - streamer.cameras[i]->TranslationVector);
        cv::Mat hReproImagePos = streamer.cameras[i]->CameraMatrix * cameraSpacePos;
        cv::Point2f reproImagePos(hReproImagePos.at<double>(0,0)/hReproImagePos.at<double>(2,0), hReproImagePos.at<double>(1,0)/hReproImagePos.at<double>(2,0));

        cv::circle(frames[i], reproImagePos, 2, cv::Scalar(0, 255, 0), 3);
    }

    dronePosition = lmOptimization(rays, dronePosition, features);
    //// drawing of optimzed reprojection /////
    for(int i = 0; i < streamer.cameraCount; i++) {
        cv::Mat cameraSpacePos = streamer.cameras[i]->RotationMatrix.inv() * (dronePosition - streamer.cameras[i]->TranslationVector);
        cv::Mat hReproImagePos = streamer.cameras[i]->CameraMatrix * cameraSpacePos;
        cv::Point2f reproImagePos(hReproImagePos.at<double>(0,0)/hReproImagePos.at<double>(2,0), hReproImagePos.at<double>(1,0)/hReproImagePos.at<double>(2,0));

        cv::circle(frames[i], reproImagePos, 2, cv::Scalar(0, 0, 255), 3);
        cv::imshow("reprojection " + std::to_string(i), frames[i]);
        cv::waitKey(1);
    }

    draw(rays, dronePosition);

    return cv::Vec3f(dronePosition.at<double>(0,0), dronePosition.at<double>(1, 0), dronePosition.at<double>(2,0));;
}

cv::Mat Triangulation::calcLeastSquarePoint(std::vector<Rayd>& rays) {

    cv::Mat I = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat M = cv::Mat::zeros(3, 3, CV_64F);
    cv::Mat P = cv::Mat::zeros(3, 1, CV_64F);

    for(int i = 0; i < streamer.cameraCount; i++) {
        Rayd ray = rays[i];

        if(ray.Origin.empty() || ray.Direction.empty())
            continue;

        cv::Mat m = ray.Direction * ray.Direction.t(); // 3x3
        cv::Mat Minc = I - m; // 3x3
        cv::Mat Pinc = Minc * ray.Origin; // 3x1

        M += Minc;
        P += Pinc;
    }

    return M.inv() * P;
}

cv::Mat Triangulation::lmOptimization(std::vector<Rayd>& rays, cv::Mat& currPos, std::vector<cv::Point2f> features) {
    double damping = 1.0;
    for(unsigned int it = 0; it < 10 && damping <= 1000000.0; it++) {
        cv::Mat JTJ = cv::Mat::zeros(3, 3, CV_64F);
        cv::Mat JTe = cv::Mat::zeros(3, 1, CV_64F);
        double squaredError = 0;

        for(int i = 0; i < streamer.cameraCount; i++) {

            if(rays[i].Origin.empty() || rays[i].Direction.empty())
                continue;

            cv::Mat cameraSpacePos = streamer.cameras[i]->RotationMatrix.inv() * (currPos - streamer.cameras[i]->TranslationVector);
            cv::Mat hReproImagePos = streamer.cameras[i]->CameraMatrix * cameraSpacePos;
            cv::Point2f reproImagePos(hReproImagePos.at<double>(0,0)/hReproImagePos.at<double>(2,0), hReproImagePos.at<double>(1,0)/hReproImagePos.at<double>(2,0));

            double e_x = cv::abs(reproImagePos.x - features[i].x);
            double e_y = cv::abs(reproImagePos.y - features[i].y);
            squaredError += e_x * e_x + e_y * e_y;

            double de_x[3];
            double de_y[3];
            for(int d = 0; d < 3; d++) {
                cv::Mat goodPositionPlus = currPos.clone();
                goodPositionPlus.at<double>(d,0) += 0.001;
                cv::Mat goodPositionMinus = currPos.clone();
                goodPositionMinus.at<double>(d,0)  -= 0.001;

                cv::Mat cameraSpacePosPlus = streamer.cameras[i]->RotationMatrix.inv() * (goodPositionPlus - streamer.cameras[i]->TranslationVector);
                cv::Mat hReproImagePosPlus = streamer.cameras[i]->CameraMatrix * cameraSpacePosPlus;
                cv::Point2f reproImagePosPlus(hReproImagePosPlus.at<double>(0,0)/hReproImagePosPlus.at<double>(2,0), hReproImagePosPlus.at<double>(1,0)/hReproImagePosPlus.at<double>(2,0));

                cv::Mat cameraSpacePosMinus = streamer.cameras[i]->RotationMatrix.inv() * (goodPositionMinus - streamer.cameras[i]->TranslationVector);
                cv::Mat hReproImagePosMinus = streamer.cameras[i]->CameraMatrix * cameraSpacePosMinus;
                cv::Point2f reproImagePosMinus(hReproImagePosMinus.at<double>(0,0)/hReproImagePosMinus.at<double>(2,0), hReproImagePosMinus.at<double>(1,0)/hReproImagePosMinus.at<double>(2,0));

                double e_xplus = cv::abs(reproImagePosPlus.x - features[i].x);
                double e_yplus = cv::abs(reproImagePosPlus.y - features[i].y);
                double e_xminus = cv::abs(reproImagePosMinus.x - features[i].x);
                double e_yminus = cv::abs(reproImagePosMinus.y - features[i].y);

                de_x[d] = (e_xplus - e_xminus) / 0.002;
                de_y[d] = (e_yplus - e_yminus) / 0.002;
            }

            for(int d = 0; d < 3; d++) {
                JTe.at<double>(d, 0) += e_x * de_x[d];
                JTe.at<double>(d, 0) += e_y * de_y[d];
            }

            for(int d1 = 0; d1 < 3; d1++) {
                for(int d2 = 0; d2 < 3; d2++) {
                    JTJ.at<double>(d1, d2) += de_x[d1] * de_x[d2];
                    JTJ.at<double>(d1, d2) += de_y[d1] * de_y[d2];
                }
            }
        }

        double newSquaredError = std::numeric_limits<double>::infinity();
        while(newSquaredError >= squaredError) {
            cv::Mat JTJdamped = JTJ;
            for(int d= 0; d < 3; d++) {
                JTJdamped.at<double>(d, d) += damping;
            }
            JTJdamped = JTJdamped.inv();
            cv::Mat delta = JTJdamped * JTe;
            cv::Mat newGoodPosition = currPos - delta;

            newSquaredError = 0.0;
            for(int i = 0; i < streamer.cameraCount; i++) {
                if(rays[i].Origin.empty() || rays[i].Direction.empty())
                    continue;

                cv::Mat cameraSpacePos = streamer.cameras[i]->RotationMatrix.inv() * (newGoodPosition - streamer.cameras[i]->TranslationVector);
                cv::Mat hReproImagePos = streamer.cameras[i]->CameraMatrix * cameraSpacePos;
                cv::Point2f reproImagePos(hReproImagePos.at<double>(0,0)/hReproImagePos.at<double>(2,0), hReproImagePos.at<double>(1,0)/hReproImagePos.at<double>(2,0));

                double e_x = cv::abs(reproImagePos.x - features[i].x);
                double e_y = cv::abs(reproImagePos.y - features[i].y);
                newSquaredError = e_x * e_x + e_y * e_y;
            }

            if(newSquaredError >= squaredError) {
                damping *= 2;
                if(damping > 1000000.0) {
                    break;
                }
            } else {
                damping *= 0.5;
                currPos = newGoodPosition;
            }
        }
    }

    return currPos;
}


void Triangulation::draw(std::vector<Rayd>& rays, cv::Vec3f dronePosition) {
    cv::Mat camOverview = cv::Mat::zeros(400, 400, CV_8UC3);
    // 400x400 should be 2x2 meter => 1m => 200 units
    float fac = 200.0f;
    cv::Point origin(200, 200);
    cv::putText(camOverview, "RC", cv::Point(3, 15) + origin, cv::FONT_HERSHEY_DUPLEX, .5, cv::Scalar(0, 0, 255), 1);

    //ref cam
    if(!rays[0].Origin.empty() && !rays[0].Direction.empty()) {
        cv::Mat refT = streamer.cameras[0]->TranslationVector;
        cv::Mat refR = streamer.cameras[0]->RotationMatrix;
        drawCamera(camOverview, refT, refR, rays[0], origin, fac, cv::Scalar(0, 0, 255));
    }

    //other cams;
    cv::Mat ct, cR;
    for(int i = 1; i < streamer.cameraCount; i++) {
        if(!rays[i].Origin.empty() && !rays[i].Direction.empty()) {
            drawCamera(camOverview, streamer.cameras[i]->TranslationVector, streamer.cameras[i]->RotationMatrix, rays[i], origin, fac);
        }
    }

    //drone pos
    cv::Point dronePos = cv::Point(dronePosition[0] * fac, dronePosition[2] * fac) + origin;
    cv::circle(camOverview, dronePos, 2, cv::Scalar(0, 255, 0), 3);
    cv::putText(camOverview, "Drone",dronePos + cv::Point(3, 15) + origin, cv::FONT_HERSHEY_DUPLEX, .5, cv::Scalar(0, 255, 0), 1);


    cv::imshow("Camera Overview", camOverview);
    cv::waitKey(1);
}

void Triangulation::drawCamera(cv::Mat& frame, cv::Mat& t, cv::Mat& R, Rayd& ray, cv::Point& origin, float fac, const cv::Scalar& color) {
    cv::Mat o = ray.Origin;
    cv::Point2f drawOrigin = cv::Point((int)(o.at<double>(0, 0) * fac), (int)(o.at<double>(2, 0) * fac)) + origin;
    cv::circle(frame, drawOrigin, 1, color, 2);

    cv::Mat forward = cv::Mat::zeros(3, 1, CV_64FC1);
    forward.at<double>(2, 0) = 1;
    forward = R * forward;
    cv::line(frame, drawOrigin, drawOrigin + cv::Point2f(forward.at<double>(0, 0), forward.at<double>(2, 0)) * 30, cv::Scalar(0, 255, 255), 1);
    cv::line(frame, drawOrigin, drawOrigin + cv::Point2f(ray.Direction.at<double>(0,0), ray.Direction.at<double>(2,0)) * 400, cv::Scalar(0, 255, 0), 1);

}
