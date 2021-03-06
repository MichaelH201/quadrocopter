#include "ICamera.h"
#include <utility>

ICamera::ICamera(int deviceId, std::string type) : ICamera(deviceId, std::move(type), std::vector<int>{1024, 576}) {}

ICamera::ICamera(int deviceId, std::string type, const std::vector<int>& imageSize) : deviceId(deviceId), camType(std::move(type)) {
    cam = new cv::VideoCapture(deviceId, cv::CAP_DSHOW);
    tracker = new DroneTracker();

    if(!cam->isOpened()) {
        std::cerr << "ERROR: Could not open camera" << std::endl;
    }

    intrinsics.ImageSize = imageSize;
    TranslationVector = cv::Mat::zeros(3, 1, CV_64FC1);
    RotationMatrix = cv::Mat::eye(3, 3, CV_64FC1);
}

ICamera::~ICamera() {
    if(IsCapturing())
        StopCapture();

    cam->release();
}

/**
 * Setup the cameras' parameters.
 */
void ICamera::Setup() {
    // prevent auto focus and set focus to maximum value
    DisableAutofocus();
    SetFocusToInfinity();
    SetResolution(intrinsics.ImageSize[0],intrinsics.ImageSize[1]);

    this->isSetup = true;
}

/**
 * Start the capturing process for this camera.
 * @param bufferIndex The pointer to the index of the buffer the frames should be written to.
 * @param multiBuffer The buffer where the frame will be written to.
 * @param mtx Mutex object which is used to lock thread sensitive actions.
 */
void ICamera::StartCapture(const int* bufferIndex, std::vector<cv::Mat>* multiBuffer, std::vector<cv::Rect>* boundingBoxBuffer, std::shared_mutex* mtx) {
    checkSetup();
    isCapturing = true;
    lastBufferIndex = *bufferIndex;

    if(displayFrames) {
        openWindow("cam" + std::to_string(deviceId));
    }

    while(IsCapturing()) {
        cv::Mat frame;
        cv::Rect boundingRect = cv::Rect();
        (*cam) >> frame;

        if(!frame.empty()) {
            if(isTrackingEnabled) {
                tracker->track(frame, boundingRect);

                if(!isDroneFocused) {
                    isDroneFocused = tracker->droneFound;
                }
            }

            // ====== thread safe ======
            mtx->lock_shared();
            int bIndex = (*bufferIndex);

            if(lastBufferIndex != bIndex) {
                SetFrameAvailable(lastBufferIndex, false);
                lastBufferIndex = bIndex;
            }

            boundingBoxBuffer->at(bIndex) = boundingRect;
            multiBuffer->at(bIndex) = frame;
            SetFrameAvailable(bIndex, true);

            mtx->unlock_shared();
            // ====== thread safe end ======

            // for debug reasons
            if(displayFrames) {
                imshow("cam" + std::to_string(deviceId), frame);
                cv::waitKey(1);
            }
        }

        frame.release();

        // sleep for a short duration
        // std::this_thread::sleep_for(std::chrono::milliseconds(5)); <<<--- nope
    }
}

void ICamera::StopCapture() {
    isCapturing = false;

    if(displayFrames)
        closeWindow("cam" + std::to_string(deviceId));
}

void ICamera::enableDroneTracking() {
    checkSetup();
    if(!isCapturing) {
        std::cerr << camType << " (device " << deviceId << ") is not capturing. Call StartCapture() before enabling drone tracking." << std::endl;
        throw std::bad_function_call();
    }

    isTrackingEnabled = true;
}

void ICamera::setCameraMatrix(cv::Mat camMat) {
    intrinsics.FocalLength = ((float)camMat.at<double>(0,0) + (float)camMat.at<double>(1,1)) / 2.0f;
    intrinsics.PrincipalPoint = std::vector<float>{(float)camMat.at<double>(0,2), (float)camMat.at<double>(1,2)};
    CameraMatrix = camMat;
}

void ICamera::setExtrinsics(cv::Mat R, cv::Mat t){
    RotationMatrix = R;
    TranslationVector = t;
}

bool ICamera::IsCapturing() {
    return isCapturing;
}

bool ICamera::IsFrameAvailable(int index) {
    if(index == 0)
        return frameAvailableB1;
    else
        return frameAvailableB2;
}

void ICamera::SetFrameAvailable(int index, bool value) {
    if(index == 0)
        frameAvailableB1 = value;
    else
        frameAvailableB2 = value;
}

void ICamera::checkSetup() {
    if(!isSetup) {
        std::cerr << camType << " is not setup. Call Setup() before using the camera." << std::endl;
        throw std::bad_function_call();
    }
}

// ===================================== //
// virtual methods
// ===================================== //

void ICamera::SetResolution(int x, int y) {
    std::cerr << "The method SetResolution(int, int) needs to be implemented in child class" << std::endl;
}

void ICamera::DisableAutofocus() {
    std::cerr << "The method DisableAutofocus() needs to be implemented in child class" << std::endl;
}

void ICamera::SetFocusToInfinity() {
    std::cerr << "The method SetFocusToInfinity() needs to be implemented in child class" << std::endl;
}

