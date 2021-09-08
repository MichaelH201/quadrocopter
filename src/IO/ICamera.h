#ifndef QUADROCOPTER_ICAMERA_H
#define QUADROCOPTER_ICAMERA_H

#include "thread"
#include "chrono"
#include "shared_mutex"
#include "opencv2/opencv.hpp"
#include "../defines.h"
#include "../scene/CameraIntrinsics.h"
#include "../DroneTracker.h"
#include "../utils.h"

class ICamera {
public:
    std::atomic<bool> isSetup = false;
    std::atomic<bool> isCapturing = false;
    std::atomic<bool> isDroneFocused = false;
    std::string camType = "ICamera";
    int deviceId;
    CameraIntrinsics intrinsics;

    cv::Mat CameraMatrix;
    cv::Mat RotationMatrix;
    cv::Mat TranslationVector;

    bool displayFrames = false;

    // constructor and destructor
    ICamera(int deviceId, std::string type);
    ICamera(int deviceId, std::string type, const std::vector<int>& imageSize);
    virtual ~ICamera();

    virtual void DisableAutofocus();
    virtual void SetFocusToInfinity();
    virtual void SetResolution(int width, int height);

    void Setup();
    void StartCapture(const int* bufferIndex, std::vector<cv::Mat>* multiBuffer, std::vector<cv::Rect>* boundingBoxBuffer, std::shared_mutex* mtx);
    void StopCapture();
    bool IsCapturing();
    bool IsFrameAvailable(int index);
    void enableDroneTracking();
    void setCameraMatrix(cv::Mat camMat);
    void setExtrinsics(cv::Mat R, cv::Mat t);

protected:
    cv::VideoCapture* cam;
    DroneTracker* tracker;

private:
    void SetFrameAvailable(int index, bool value);
    std::atomic<bool> frameAvailableB1 = false;
    std::atomic<bool> frameAvailableB2 = false;
    std::atomic<bool> isTrackingEnabled = false;

    int lastBufferIndex = 0;

    void checkSetup();
};
#endif //QUADROCOPTER_ICAMERA_H
