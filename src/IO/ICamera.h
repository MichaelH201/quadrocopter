#ifndef QUADROCOPTER_ICAMERA_H
#define QUADROCOPTER_ICAMERA_H

#include "opencv2/opencv.hpp"
#include "concurrent_queue.h"
#include "../defines.h"

using namespace concurrency;

class ICamera {
public:
    bool isSetup = false;
    std::string camType = "ICamera";

    // constructor and destructor
    explicit ICamera(int deviceId, std::string  type);
    virtual ~ICamera();

    virtual void DisableAutofocus();
    virtual void SetFocusToInfinity();

    void Setup();
    void StartCapture(concurrent_queue<cv::Mat>* queue);
    void StopCapture();

    bool IsCapturing();

protected:
    cv::VideoCapture* cam;

private:
    int deviceId;
    std::atomic<bool> isCapturing = false;

    void checkSetup();
};
#endif //QUADROCOPTER_ICAMERA_H
