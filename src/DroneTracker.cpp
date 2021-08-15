#include "DroneTracker.h"

void DroneTracker::track(cv::Mat& frame) {
    // do this until any key is pressed
    cv::Mat grayScale;
    cvtColor(frame, grayScale, cv::COLOR_BGR2GRAY);

    if(buffer.size() < MAX_BUFFER_LENGTH) {
        buffer.push_back(grayScale);
        return;
    } else {
        buffer.pop_front();
        buffer.push_back(grayScale);
    }

    for(cv::Rect2f& bb : detectMovingObjects(frame)) {
        cv::rectangle(frame, bb, cv::Scalar(255, 0, 0));
    }
}

std::vector<cv::Rect2f> DroneTracker::detectMovingObjects(cv::Mat &img) {
    cv::Mat diff;
    cv::absdiff(buffer[MAX_BUFFER_LENGTH-2], buffer[MAX_BUFFER_LENGTH-1],diff);
    cv::threshold(diff, diff, 15, 255, cv::THRESH_BINARY);
    cv::dilate(diff, diff, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6), cv::Point(3,3)));

    // find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(diff, contours, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);

    std::vector<cv::Rect2f> boundingRects;
    for(auto & contour : contours) {
        if(cv::contourArea(contour) >= 100)
            boundingRects.push_back(cv::boundingRect(contour));
    }

    combineOverlappingBoundingBoxes(boundingRects);

    return boundingRects;
}

void DroneTracker::combineOverlappingBoundingBoxes(std::vector<cv::Rect2f>& bboxes) {
    bool dirty = true;

    while(dirty) {
        dirty = false;

        for(int i = 0; i < bboxes.size(); i++) {
            for(int j = 0; j < bboxes.size(); j++) {
                if(i == j) continue;

                if(containsRect(bboxes[i], bboxes[j])) {
                    bboxes[i] = combineRects(bboxes[i], bboxes[j]);
                    bboxes.erase(bboxes.begin()+j);
                    j--;
                    if(j < i) { i--; }
                    dirty = true;
                }
            }
        }
    }
}

bool DroneTracker::containsRect(const cv::Rect2f& rect1, const cv::Rect2f& rect2) {
    return rect1.contains(rect2.tl()) || rect1.contains(rect2.br());
}

cv::Rect2f DroneTracker::combineRects(const cv::Rect2f& rect1, const cv::Rect2f& rect2) {
    return cv::boundingRect(std::vector<cv::Point>{rect1.tl(), rect1.br(), rect2.tl(), rect2.br()});
}

