#include "Cameras.h"

using namespace std;
using namespace cv;

depthCamera::depthCamera(int camNum, int width, int height, int fps)
    : pipe{}, cfg{}, colorFrame{cv::Size(width, height), CV_8UC3}, grayFrame{
                                                                       cv::Size(width, height),
                                                                       CV_8UC1}
{
    setCamParams(323, 245, 608, 608);
    setDistCoeffs();
    this->width = width;
    this->height = height;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_ANY, fps);

    // Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);
}

depthCamera::~depthCamera()
{
}

void depthCamera::setManualExposure(int exposuretime)
{
    // TODO impl
}

void depthCamera::setAutoExposure()
{
    // TODO impl
}

void depthCamera::getFrame()
{
    rs2::frameset frames;
    frames = pipe.wait_for_frames();
    rs2::video_frame frame = frames.get_color_frame();

    colorFrame.data = (uint8_t *)frame.get_data();
    cv::cvtColor(colorFrame, grayFrame, cv::COLOR_BGR2GRAY);
}
