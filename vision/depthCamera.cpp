#include "Cameras.h"

using namespace std;

depthCamera::depthCamera(int camNum, int width, int height, int fps) : pipe{}, cfg{}
{
    setCamParams(608, 608, 323, 245);
    setDistCoeffs();

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

cv::Mat depthCamera::getFrame()
{
    rs2::frameset frames;
    frames = pipe.wait_for_frames();

    cv::Mat matframe(cv::Size(640, 480), CV_8UC3);

    rs2::video_frame frame = frames.get_color_frame();

    matframe.data = (uint8_t *)frame.get_data();

    return matframe;
}
