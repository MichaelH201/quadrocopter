#include "DroneTracker.h"

DroneTracker::DroneTracker(CameraStreamer& streamer) : streamer(streamer) {
    for(int i = 0; i < streamer.cameraCount; i++){
        openWindow("cam" + std::to_string(i) + " tracked");
        openWindow("cam" + std::to_string(i) + " diff");
    }
}

void DroneTracker::track() {
    createBackgroundModel();

    // do this until any key is pressed
    while(true) {
        cv::Mat frame = streamer.GetFrame(0);
        cv::Mat grayScale;
        cvtColor(frame, grayScale, cv::COLOR_BGR2GRAY);

        if(buffer.size() < MAX_BUFFER_LENGTH) {
            buffer.push_back(grayScale);
            continue;
        } else {
            buffer.pop_front();
            buffer.push_back(grayScale);
        }

        for(cv::Rect2f& bb : detectMovingObjects(frame)) {
            cv::rectangle(frame, bb, cv::Scalar(255, 0, 0));
        }

        cv::imshow("cam" + std::to_string(0) + " tracked", frame);
        cv::waitKey(1);


        int k = cv::waitKey(33);
        if(k == 27)
            break;
    }
}

std::vector<cv::Rect2f> DroneTracker::detectMovingObjects(cv::Mat &img) {

    //cv::Mat diffSum = cv::Mat::zeros(cv::Size(1280, 720), CV_8UC1);
    cv::Mat diff;
    cv::absdiff(buffer[MAX_BUFFER_LENGTH-2], buffer[MAX_BUFFER_LENGTH-1],diff);
    cv::threshold(diff, diff, 30, 255, cv::THRESH_BINARY);
    cv::dilate(diff, diff, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6), cv::Point(3,3)));

    // find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(diff, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    std::vector<cv::Rect2f> boundingRects;
    for(auto & contour : contours) {
        if(cv::contourArea(contour) >= 100)
            boundingRects.push_back(cv::boundingRect(contour));
    }

    cv::imshow("cam" + std::to_string(0) + " diff", diff);
    cv::waitKey(1);

    return boundingRects;
}

void DroneTracker::createBackgroundModel() {
    std::cout << "Creating the background model for drone detection..." << std::endl;

    std::cout << "Press a key to captures the background Image" << std::endl;
    if(cv::waitKey(0) >= 0) {
        cvtColor(streamer.GetFrame(0), backgroundModel, cv::COLOR_BGR2GRAY);
    }
}

