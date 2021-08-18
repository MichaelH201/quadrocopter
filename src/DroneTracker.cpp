#include "DroneTracker.h"

DroneTracker::DroneTracker() {
    tracker = cv::TrackerKCF::create();
}

DroneTracker::~DroneTracker() {
    delete tracker;
}


void DroneTracker::track(cv::Mat& frame) {
    cv::Mat grayScale;
    cvtColor(frame, grayScale, cv::COLOR_BGR2GRAY);

    double minVal, maxVal;
    cv::minMaxLoc(frame, &minVal, &maxVal);
    frame = ((frame - minVal) / (maxVal - minVal)) * 255;

    if(buffer.size() < MAX_BUFFER_LENGTH) {
        buffer.push_back(grayScale);
        return;
    } else {
        buffer.pop_front();
        buffer.push_back(grayScale);
    }

    auto bbs = detectMovingObjects();

    if(!droneFound) {
        tryDetectDrone(frame, bbs);
    }
    else {
        tracker->update(frame, currentBbox);
    }

    drawBoundingBoxes(frame, bbs);

    /*
    float bestDist = std::numeric_limits<float>::max();
    int index = -1;
    for(int i = 0; i < bbs.size(); i++) {
        cv::Point c1 = (currentBbox->br() + currentBbox->tl()) * 0.5f;
        cv::Point c2 = (bbs[i].br() + bbs[i].tl()) * 0.5f;
        auto dist = (float)cv::sqrt((c1.x - c2.x)*(c1.x - c2.x) + (c1.y - c2.y)*(c1.y - c2.y));

        if(dist < bestDist) {
            float areaDelta = cv::abs(currentBbox->area() / bbs[i].area());
            if(areaDelta > .8 && areaDelta < 1.2) {
                bestDist = dist;
                index = i;
            }
        }
    }
    if(index >= 0)
        currentBbox = new cv::Rect2f(bbs[index].x, bbs[index].y, bbs[index].width, bbs[index].height);
        */
}

bool DroneTracker::tryDetectDrone(cv::Mat& frame, std::vector<cv::Rect2f>& bbs) {
    if(bbs.size() == 1 && bbs[0].area() > 5000 && bbs[0].area() < 40000) {
        float ratio = bbs[0].height/bbs[0].width;
        if(ratio > .3 && ratio < .6) {
            currentBbox = bbs[0];
            tracker->init(frame, currentBbox);
            droneFound = true;
            return true;
        }
    }

    return false;
}

bool DroneTracker::tryGetBoundingBox(cv::Rect2f& rect) {
    if(droneFound && currentBbox.area() > 0) {
        rect = currentBbox;
        return true;
    }

    return false;
}

std::vector<cv::Rect2f> DroneTracker::detectMovingObjects() {
    cv::Mat fgMask, addition;

    //backSub->apply(img.clone(), fgMask);
    //cv::morphologyEx(fgMask, fgMask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

    cv::absdiff(buffer[MAX_BUFFER_LENGTH-2], buffer[MAX_BUFFER_LENGTH-1], fgMask);
    cv::blur(fgMask, fgMask, cv::Size(3, 3));
    cv::threshold(fgMask, fgMask, 25, 255, cv::THRESH_BINARY);
    cv::dilate(fgMask, fgMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(6, 6), cv::Point(3,3)));

    cv::imshow("diff image", fgMask);

    // find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fgMask, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Rect2f> boundingRects;
    for(auto & contour : contours) {
        cv::Rect2f bb = cv::boundingRect(contour);
        if(bb.area() >= 100) {
            boundingRects.push_back(bb);
        }
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

                if(minimalRectDistance(bboxes[i], bboxes[j]) < 25) {
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

void DroneTracker::drawBoundingBoxes(cv::Mat& img, std::vector<cv::Rect2f>& bbs) {
    cv::Rect2f droneBB;
    if(tryGetBoundingBox(droneBB)) {
        cv::rectangle(img, droneBB, cv::Scalar(0, 0, 255));
        cv::putText(img, "Drone", droneBB.tl() - cv::Point2f(0, 5), cv::FONT_HERSHEY_DUPLEX, .5, cv::Scalar(0, 0, 255), 1);
    }

    for(cv::Rect2f& bb : bbs) {
        if(bb == droneBB) continue;
        cv::rectangle(img, bb, cv::Scalar(255, 0, 0));
    }
}

float DroneTracker::minimalRectDistance(const cv::Rect2f& rect1, const cv::Rect2f& rect2) {

    if(rect1.contains(rect2.tl()) || rect1.contains(rect2.br()))
        return 0.0f;

    float minX, minY;
    float x_tltl, x_tlbr, x_brtl, x_brbr;
    float y_tltl, y_tlbr, y_brtl, y_brbr;

    x_tltl = cv::abs(rect1.tl().x - rect2.tl().x);
    x_tlbr = cv::abs(rect1.tl().x - rect2.br().x);
    x_brtl = cv::abs(rect1.br().x - rect2.tl().x);
    x_brbr = cv::abs(rect1.br().x - rect2.br().x);

    y_tltl = cv::abs(rect1.tl().y - rect2.tl().y);
    y_tlbr = cv::abs(rect1.tl().y - rect2.br().y);
    y_brtl = cv::abs(rect1.br().y - rect2.tl().y);
    y_brbr = cv::abs(rect1.br().y - rect2.br().y);

    minX = cv::min(x_tltl, cv::min(x_tlbr, cv::min(x_brtl, x_brbr)));
    minY = cv::min(y_tltl, cv::min(y_tlbr, cv::min(y_brtl, y_brbr)));
    return (minX + minY) / 2;

}

cv::Rect2f DroneTracker::combineRects(const cv::Rect2f& rect1, const cv::Rect2f& rect2) {
    return cv::boundingRect(std::vector<cv::Point>{rect1.tl(), rect1.br(), rect2.tl(), rect2.br()});
}
