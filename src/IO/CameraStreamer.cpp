#include "CameraStreamer.h"

CameraStreamer::CameraStreamer(std::vector<int> deviceIds, bool debugMode) {
    cameraCount = deviceIds.size();
    bufferIndex = new int(0);
    mtx = new std::shared_mutex();

    ICamera* cam;
    std::thread* t;
    std::vector<cv::Mat>* fq;
    std::vector<cv::Rect>* bq;

    for(int i = 0; i < cameraCount; i++) {
        // create and setup camera
        int dId = deviceIds[i];
        cam = new LogitechC920(dId);
        cam->Setup();

        if(debugMode)
            cam->displayFrames = true;

        // add camera to vector
        cameras.push_back(cam);

        // create queue and add it to vector
        fq = new std::vector<cv::Mat>(2);
        bq = new std::vector<cv::Rect>(2);
        frame_queues.push_back(fq);
        bounding_queues.push_back(bq);

        // create thread and add it to vector
        t = new std::thread(&ICamera::StartCapture, cam, bufferIndex, fq, bq, mtx);
        camera_threads.push_back(t);

        std::cout << "Start capturing with " << cam->camType << " (DeviceId: " << dId << ")" << std::endl;
    }

    // make sure every camera is setup and capturing before move on to next step
    std::cout << "Wait for all cameras to setup..." << std::endl;
    while(true) {
        for(int i = 0; i < cameraCount; i++) {
            if(!cameras[i]->isCapturing) {
                goto waitAndCheck;
            }
        }
        break;
        waitAndCheck:;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "Camera setup done." << std::endl;
}

CameraStreamer::~CameraStreamer() {
    for(int i = 0; i < cameraCount; i++) {
        cameras[i]->StopCapture();
        camera_threads[i]->join();

        std::cout << "Finished capturing with " << cameras[i]->camType << " (DeviceId: " << cameras[i]->deviceId << ")" << std::endl;

        delete cameras[i];
        delete frame_queues[i];
        delete bounding_queues[i];
        delete camera_threads[i];
    }

    delete bufferIndex;
    delete mtx;
}

void CameraStreamer::GetFrames(std::vector<cv::Mat>& frames) {
    frames = std::vector<cv::Mat>();
    int readIndex = *bufferIndex;

    while(true) {
        // check if every camera has a new frame in the active buffer.
        for(int i = 0; i < cameraCount; i++) {
            if(!cameras[i]->IsFrameAvailable(readIndex)) {
                goto endloop;
            }
        }

        mtx->lock();
        *bufferIndex = (readIndex+1) % 2;
        mtx->unlock();

        // get the frames from all queues
        for(int i = 0; i < cameraCount; i++) {
            cv::Mat frame = (*frame_queues[i])[readIndex];
            frames.push_back(frame);
        }

        break;
        endloop:;
    }
}

void CameraStreamer::GetFrames(std::vector<cv::Mat>& frames, std::vector<cv::Rect>& bbs) {
    frames = std::vector<cv::Mat>();
    int readIndex = *bufferIndex;

    while(true) {
        // check if every camera has a new frame in the active buffer.
        for(int i = 0; i < cameraCount; i++) {
            if(!cameras[i]->IsFrameAvailable(readIndex)) {
                goto endloop;
            }
        }

        mtx->lock();
        *bufferIndex = (readIndex+1) % 2;
        mtx->unlock();

        // get the frames from all queues
        for(int i = 0; i < cameraCount; i++) {
            cv::Mat frame = (*frame_queues[i])[readIndex];
            frames.push_back(frame);
            cv::Rect bb = (*bounding_queues[i])[readIndex];
            bbs.push_back(bb);
        }

        break;
        endloop:;
    }
}

cv::Mat CameraStreamer::GetFrame(int camIndex) {
    if(camIndex < 0 || cameras.size() <= camIndex)
        throw std::invalid_argument("camIndex " + std::to_string(camIndex) + " is out of range. Camera array size: " +
                                            std::to_string(cameras.size()));

    int readIndex = *bufferIndex;

    while(true) {
        if(!cameras[camIndex]->IsFrameAvailable(readIndex)) {
            continue;
        }

        mtx->lock();
        *bufferIndex = (readIndex+1) % 2;
        mtx->unlock();

        break;
    }

    return (*frame_queues[camIndex])[readIndex];
}

/**
 * Activates the drone tracking for all cameras. This will lead to firstly a detection and
 * a tracking afterwards.
 */
void CameraStreamer::activateDroneTracking() {
    std::cout << "Detecting drone ..." << std::endl;

    for(int i = 0; i < cameraCount; i++) {
        cameras[i]->enableDroneTracking();
    }

    // make sure every camera found the drone before we can proceed
    while(true) {
        for(int i = 0; i < cameraCount; i++) {
            if(!cameras[i]->isDroneFocused) {
                goto repeat;
            }
        }
        break;
        repeat:;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "Detection done." << std::endl;
}
