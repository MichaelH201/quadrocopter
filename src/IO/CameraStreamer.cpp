#include "CameraStreamer.h"

CameraStreamer::CameraStreamer(std::vector<int> deviceIds) {
    camera_count = deviceIds.size();

    ICamera* cam;
    std::thread* t;
    concurrent_queue<cv::Mat>* q;

    for(int i = 0; i < camera_count; i++) {
        // create and setup camera
        int dId = deviceIds[i];
        cam = new LogitechC920(dId);
        cam->Setup();

        // add camera to vector
        cameras.push_back(cam);

        // create queue and add it to vector
        q = new concurrent_queue<cv::Mat>;
        frame_queues.push_back(q);

        // create thread and add it to vector
        t = new std::thread(&ICamera::StartCapture, cam, q);
        camera_threads.push_back(t);

        std::cout << "Start capturing with " << cam->camType << " (DeviceId: " << dId << ")" << std::endl;
    }
}

CameraStreamer::~CameraStreamer() {
    for(int i = 0; i < camera_count; i++) {
        cameras[i]->StopCapture();
        camera_threads[i]->join();

        std::cout << "Finished capturing with " << cameras[i]->camType << " (DeviceId: " << i << ")" << std::endl;

        delete cameras[i];
        delete frame_queues[i];
        delete camera_threads[i];


    }
}
