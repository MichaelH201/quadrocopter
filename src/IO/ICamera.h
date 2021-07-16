#ifndef QUADROCOPTER_ICAMERA_H
#define QUADROCOPTER_ICAMERA_H

#include "thread"
#include "chrono"
#include "shared_mutex"
#include "opencv2/opencv.hpp"
#include "../defines.h"
#include "../scene/CameraIntrinsics.h"

class ICamera {
public:
    bool isSetup = false;
    std::string camType = "ICamera";
    int deviceId;
    CameraIntrinsics<double> intrinsics;

    // constructor and destructor
    ICamera(int deviceId, std::string type);
    ICamera(int deviceId, std::string type, base::Vec2d imageSize);
    virtual ~ICamera();

    virtual void DisableAutofocus();
    virtual void SetFocusToInfinity();
    virtual void SetResolution(int width, int height);

    void Setup();
    void StartCapture(const int* bufferIndex, std::vector<cv::Mat>* multiBuffer, std::shared_mutex* mtx);
    void StopCapture();
    bool IsCapturing();
    bool IsFrameAvailable(int index);
    void SetFrameAvailable(int index, bool value);

protected:
    cv::VideoCapture* cam;

private:
    std::atomic<bool> isCapturing = false;
    std::atomic<bool> frameAvailableB1 = false;
    std::atomic<bool> frameAvailableB2 = false;

    void checkSetup();
};
#endif //QUADROCOPTER_ICAMERA_H
