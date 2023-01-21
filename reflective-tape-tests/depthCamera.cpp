#include "Cameras.h"

using namespace std;
using namespace cv;

depthCamera::depthCamera(string camSerial, int width, int height, int fps)
    : pipe{}, cfg{}, colorFrame{cv::Size(width, height), CV_8UC3},
      grayFrame{cv::Size(width, height), CV_8UC1}, depthFrame{cv::Size(width, height), CV_8UC3}
{
    // Blue camera
    // setCamParams(608, 608, 323, 245);
    // setDistCoeffs(0.09116903370720442, 0.2567349843314421, -0.003936586357063021,
    // 0.001658039412119442, -1.633408316803933);
    // setDistCoeffs(0, 0, 0, 0, 0);
    // Red camera
    setCamParams(599, 600, 334, 236);
    this->width = width;
    this->height = height;

    // Select camera by serial number
    cfg.enable_device(camSerial);

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_ANY, fps);

    // Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile prof = pipe.start(cfg);

    // Disgusting one-liner to disable laser
    prof.get_device().first<rs2::depth_sensor>().set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
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

    rs2::colorizer color_map;
    rs2::frame dpframe = frames.get_depth_frame().apply_filter(color_map);
    depthFrame.data = (uint8_t *)(dpframe.get_data());
}
