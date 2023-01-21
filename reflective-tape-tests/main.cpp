#include "main.h"
#include "Cameras.h"

using namespace std;
using namespace cv;

int main(void)
{
    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration
    pipe.start();

    const auto window_name = "Display Image";
    namedWindow(window_name, WINDOW_AUTOSIZE);

    while (waitKey(1) < 0 && getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
        rs2::frame depth = data.get_depth_frame();
        rs2::depth_frame dpframe(depth);
        rs2::frame depthColored = depth.apply_filter(color_map);

        // Query frame size (width and height)
        const int w = depth.as<rs2::video_frame>().get_width();
        const int h = depth.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        Mat color(Size(w, h), CV_8UC3, (void *)depthColored.get_data(), Mat::AUTO_STEP);
        Mat nocolor(Size(w, h), CV_16UC1, (void *)dpframe.get_data(), Mat::AUTO_STEP);

        nocolor.convertTo(nocolor, CV_8UC1, 15 / 256.0);

        cout << nocolor.at<Vec3f>(300, 200) << endl;
        cout << dpframe.get_distance(300, 200) << endl;

        // Update the window with new data
        imshow(window_name, color);
    }

    /*
    while (true)
    {
        Mat frame = depth.colorFrame;
        Mat gray = depth.grayFrame;

        Mat black = Mat::zeros(Size(gray.cols, gray.rows), CV_8UC1);

        Mat depthFrame = depth.depthFrame.clone();

        Mat dark_gray;
        addWeighted(gray, 0.5, black, 0.5, -50, dark_gray);

        vector<Mat> gray_frames;
        split(frame, gray_frames);

        Mat bin_frame;

        threshold(gray, bin_frame, thresh, 255, THRESH_BINARY);

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        findContours(bin_frame, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);

        drawContours(frame, contours, -1, Scalar(0, 255, 0), 2);

        for (uint8_t i = 0; i < contours.size(); i++)
        {
            Moments mmnts = moments(i);
            int cx = (int)mmnts.m10 / mmnts.m00;
            int cy = (int)mmnts.m01 / mmnts.m00;
        }

        cout << depthFrame.at<Vec3f>(300, 200) << endl;

        // imshow("depth", depthFrame);
        imshow("frame", frame);
        waitKey(1);
    }
        */
}
