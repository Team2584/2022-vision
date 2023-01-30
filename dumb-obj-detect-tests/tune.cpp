#include <cmath>
#include <iomanip>
#include <iostream>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>
#include <sstream>
#include <opencv2/opencv.hpp>

#define WIDTH 640
#define HEIGHT 480
#define FPS 60

#define DEPTH_BLUE "017322071728"
#define DEPTH_RED "939622072805"

using namespace std;
using namespace cv;

int main(void)
{
    rs2::pipeline pipe;
    rs2::config cfg;

    // Select camera by serial number
    cfg.enable_device(DEPTH_BLUE);

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);

    // Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile prof = pipe.start(cfg);

    // Disgusting one-liner to disable laser
    prof.get_device().first<rs2::depth_sensor>().set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    int min_hue = 8;
    int max_hue = 30;
    int min_sat = 200;
    int max_sat = 255;
    int min_val = 120;
    int max_val = 255;

    namedWindow("Color Image");
    namedWindow("Detection");
    namedWindow("Hue Mask");
    namedWindow("Saturation Mask");
    namedWindow("Value Mask");

    // GUI STUFF
    createTrackbar("Min Hue", "Hue Mask", &min_hue, 179);
    createTrackbar("Max Hue", "Hue Mask", &max_hue, 179);

    createTrackbar("Min Saturation", "Saturation Mask", &min_sat, 255);
    createTrackbar("Max Saturation", "Saturation Mask", &max_sat, 255);

    createTrackbar("Min Value", "Value Mask", &min_val, 255);
    createTrackbar("Max Value", "Value Mask", &max_val, 255);

    while (true)
    {

        rs2::frameset frames;
        frames = pipe.wait_for_frames();
        rs2::video_frame frame = frames.get_color_frame();

        Mat colorFrame(Size(WIDTH, HEIGHT), CV_8UC3, (void *)frame.get_data());

        GaussianBlur(colorFrame, colorFrame, Size(17, 17), 1.2, 1.2, BORDER_DEFAULT);

        Mat hsvFrame;
        cvtColor(colorFrame, hsvFrame, COLOR_BGR2HSV);
        Mat channels[3];
        split(hsvFrame, channels);

        Mat mask;
        Mat colorMask;
        inRange(hsvFrame, Scalar(min_hue, min_sat, min_val), Scalar(max_hue, max_sat, max_val),
                mask);
        cvtColor(mask, colorMask, COLOR_GRAY2BGR);

        /* Filter tuning masks */
        Mat hueMask;
        inRange(channels[0], min_hue, max_hue, hueMask);
        Mat satMask;
        inRange(channels[1], min_sat, max_sat, satMask);
        Mat valMask;
        inRange(channels[2], min_val, max_val, valMask);

        Mat justCone(Size(WIDTH, HEIGHT), CV_8UC1);
        bitwise_and(colorFrame, colorMask, justCone);

        int box_size = 50;
        box_size /= 2;
        Rect rbox = Rect(WIDTH / 2 - 10, HEIGHT / 2 - 10, box_size, box_size);
        Mat box = Mat(colorFrame, rbox);

        Scalar hue = mean(box);
        cout << "Hue in box: " << hue[0] << endl
             << "Saturation in box: " << hue[1] << endl
             << "Value in box : " << hue[2] << endl
             << endl;

        rectangle(colorFrame, rbox, Scalar(255, 255, 255), 2);

        imshow("Color Image", colorFrame);
        // imshow("hsv", hsvFrame);
        imshow("Hue Mask", hueMask);
        imshow("Saturation Mask", satMask);
        imshow("Value Mask", valMask);
        // imshow("mask", mask);
        imshow("Detection", justCone);

        moveWindow("Color Image", 0, 0);
        moveWindow("Detection", getWindowImageRect("Color Image").width + 5, 0);

        moveWindow("Hue Mask", 0, getWindowImageRect("Color Image").height + 30);

        moveWindow("Saturation Mask", getWindowImageRect("Hue Mask").width + 5,
                   getWindowImageRect("Color Image").height + 30);

        moveWindow("Value Mask",
                   getWindowImageRect("Hue Mask").width + getWindowImageRect("Value Mask").width +
                       10,
                   getWindowImageRect("Color Image").height + 30);
        // imshow("hue", channels[0]);
        // imshow("saturation", channels[1]);
        // imshow("value", channels[2]);

        if (waitKey(1) == 'q')
            break;

        colorFrame.release();
        hsvFrame.release();
        colorMask.release();
        hueMask.release();
        valMask.release();
        satMask.release();
        justCone.release();
        box.release();
    }
    destroyAllWindows();

    cout << "Params at exit:" << endl;
    cerr << "min_hue: " << min_hue << endl;
    cerr << "max_hue: " << max_hue << endl;
    cerr << "min_sat: " << min_sat << endl;
    cerr << "max_sat: " << max_sat << endl;
    cerr << "min_val: " << min_val << endl;
    cerr << "max_val: " << max_val << endl;
}
