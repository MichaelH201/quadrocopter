#include "CameraStreamer.h"

CameraStreamer::CameraStreamer(std::vector<int> deviceIds, bool debugMode) {
    cameraCount = deviceIds.size();
    bufferIndex = new int(0);
    mtx = new std::shared_mutex();

    ICamera* cam;
    std::thread* t;
    std::vector<cv::Mat>* q;

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
        q = new std::vector<cv::Mat>;
        q->push_back(cv::Mat());
        q->push_back(cv::Mat());
        frame_queues.push_back(q);

        // create thread and add it to vector
        t = new std::thread(&ICamera::StartCapture, cam, bufferIndex, q, mtx);
        camera_threads.push_back(t);

        std::cout << "Start capturing with " << cam->camType << " (DeviceId: " << dId << ")" << std::endl;
    }
}

CameraStreamer::~CameraStreamer() {
    for(int i = 0; i < cameraCount; i++) {
        cameras[i]->StopCapture();
        camera_threads[i]->join();

        std::cout << "Finished capturing with " << cameras[i]->camType << " (DeviceId: " << cameras[i]->deviceId << ")" << std::endl;

        delete cameras[i];
        delete frame_queues[i];
        delete camera_threads[i];
    }

    delete bufferIndex;
    delete mtx;
}

bool CameraStreamer::TryGetFrames(std::vector<cv::Mat>* frames) {
    frames->clear();
    int readIndex = *bufferIndex;

    // check if every camera has a new frame in the active buffer.
    for(int i = 0; i < cameraCount; i++) {
        if(!cameras[i]->IsFrameAvailable(readIndex)) {
            return false;
        }
    }

    mtx->lock();
    *bufferIndex = (readIndex+1) % 2;
    mtx->unlock();

    // get the frames from all queues and flag the buffer to not contain new images
    for(int i = 0; i < cameraCount; i++) {
        cv::Mat frame;
        frames->push_back((*frame_queues[i])[readIndex]);
        cameras[i]->SetFrameAvailable(readIndex, false);
    }

    return true;
}
