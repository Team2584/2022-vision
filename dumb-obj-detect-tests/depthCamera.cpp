#include "Cameras.h"

using namespace std;
using namespace cv;

depthCamera::depthCamera(string camSerial, int width, int height, int fps)
    : pipe{}, cfg{}, colorFrame{cv::Size(width, height), CV_8UC3}, grayFrame{
                                                                       cv::Size(width, height),
                                                                       CV_8UC1}
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
}

static void on_trackbar(int, void *)
{
}

void depthCamera::findCones()
{
    GaussianBlur(colorFrame, colorFrame, Size(17, 17), 1.2, 1.2, BORDER_DEFAULT);

    Mat hsvFrame;
    cvtColor(colorFrame, hsvFrame, COLOR_BGR2HSV);

    uint8_t min_hue = 8;
    uint8_t max_hue = 30;
    uint8_t min_sat = 200;
    uint8_t max_sat = 255;
    uint8_t min_val = 120;
    uint8_t max_val = 255;

    Mat channels[3];
    split(hsvFrame, channels);

    Mat mask;
    Mat colorMask;
    inRange(hsvFrame, Scalar(min_hue, min_sat, min_val), Scalar(max_hue, max_sat, max_val), mask);
    cvtColor(mask, colorMask, COLOR_GRAY2BGR);

    /* Filter tuning masks
    Mat hueMask;
    inRange(channels[0], min_hue, max_hue, hueMask);
    Mat satMask;
    inRange(channels[1], min_sat, max_sat, satMask);
    Mat valMask;
    inRange(channels[2], min_val, max_val, valMask);
    */

    Mat justCone(Size(width, height), CV_8UC1);
    bitwise_and(colorFrame, colorMask, justCone);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<Rect> boundBoxes;
    for (int i = 0; i < contours.size(); i++)
    {
        Rect boundBox = boundingRect(contours[i]);
        if (boundBox.width < 10 || boundBox.height < 10) // Discard small detections
            continue;

        boundBoxes.push_back(boundBox);
        rectangle(colorFrame, boundBox, Scalar(255, 0, 0));
        drawContours(colorFrame, contours, i, Scalar(0, 255, 0), 2, FILLED);
    }


    // Trackbars for range
    /*
    char tbname_min[50];
    snprintf(tbname_min, sizeof(tbname_min), "Min Hue Value %d", 179);
    createTrackbar(tbname_min, "rgb", &min_hue, 179, on_trackbar);

    char tbname_max[50];
    snprintf(tbname_max, sizeof(tbname_max), "Min Hue Value %d", 179);
    createTrackbar(tbname_max, "rgb", &max_hue, 179, on_trackbar);
    */

    /*
    int box_size = 50;
    box_size /= 2;

    Mat box = Mat(hsvFrame, Rect(width / 2 - 10, height / 2 - 10, box_size, box_size));

    Scalar hue = mean(box);
    cout << "Saturation in box: " << hue[1] << endl << "Value in box: " << hue[2] << endl;
    // 7-25

    // Display stuff
    line(colorFrame, Point(width / 2 - box_size, height / 2 - box_size),
         Point(width / 2 + box_size, height / 2 - box_size), Scalar(255, 255, 255), 2);

    line(colorFrame, Point(width / 2 - box_size, height / 2 + box_size),
         Point(width / 2 + box_size, height / 2 + box_size), Scalar(255, 255, 255), 2);

    line(colorFrame, Point(width / 2 - box_size, height / 2 - box_size),
         Point(width / 2 - box_size, height / 2 + box_size), Scalar(255, 255, 255), 2);

    line(colorFrame, Point(width / 2 + box_size, height / 2 - box_size),
         Point(width / 2 + box_size, height / 2 + box_size), Scalar(255, 255, 255), 2);
         */

    imshow("rgb", colorFrame);
    // imshow("hsv", hsvFrame);
    // imshow("hue mask", hueMask);
    // imshow("saturation mask", satMask);
    // imshow("value mask", valMask);
    // imshow("mask", mask);
    // imshow("cone", justCone);
    // imshow("saturation", channels[1]);
    // imshow("value", channels[2]);
}
