#ifndef QUADROCOPTER_ICAMERA_H
#define QUADROCOPTER_ICAMERA_H

#include "opencv2/opencv.hpp"
#include "../defines.h"

class ICamera {
public:
    explicit ICamera(int deviceId, std::string  type);
    virtual ~ICamera();

    virtual cv::Mat getFrame();
    virtual void DisableAutofocus();
    virtual void SetFocusToInfinity();

    void Setup();
    void StartCapture();
    void StopCapture();

    bool IsCapturing();

    bool isSetup = false;
    bool isCapturing = false;
    std::string camType = "ICamera";

protected:
    cv::VideoCapture cam;

private:
    void checkSetup();
};
#endif //QUADROCOPTER_ICAMERA_H
