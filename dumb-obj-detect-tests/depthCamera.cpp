#include "Cameras.h"

using namespace std;
using namespace cv;

depthCamera::depthCamera(string camSerial, int width, int height, int fps)
    : pipe{}, cfg{}, colorFrame{cv::Size(width, height), CV_8UC3},
      grayFrame{cv::Size(width, height), CV_8UC1}, depthFrame{nullptr}
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
    depthFrame = frames.get_depth_frame();

    colorFrame.data = (uint8_t *)frame.get_data();
    cv::cvtColor(colorFrame, grayFrame, cv::COLOR_BGR2GRAY);
}

double depthCamera::get_distance(int x, int y)
{
    return depthFrame.get_units() * reinterpret_cast<const uint16_t *>(
                                        depthFrame.get_data())[y * depthFrame.get_width() + x];
}

std::pair<double, double> depthCamera::findCones()
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

    /* Filter tuning masks */
    Mat hueMask;
    inRange(channels[0], min_hue, max_hue, hueMask);
    Mat satMask;
    inRange(channels[1], min_sat, max_sat, satMask);
    Mat valMask;
    inRange(channels[2], min_val, max_val, valMask);

    Mat justCone(Size(width, height), CV_8UC1);
    bitwise_and(colorFrame, colorMask, justCone);

    // Get contours of cones
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Show contours & bounding boxes
    vector<Rect> boundBoxes;
    vector<vector<Point>> goodContours;
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        Rect boundBox = boundingRect(contours[i]);
        if (boundBox.width < 10 || boundBox.height < 10) // Discard small detections
            continue;

        goodContours.push_back(contours[i]);
        boundBoxes.push_back(boundBox);
        drawContours(colorFrame, contours, i, Scalar(0, 255, 0), 2, FILLED);
        rectangle(colorFrame, boundBox, Scalar(255, 0, 0));
    }

    if (boundBoxes.size() == 0)
        return pair(0, 0);

    // For now just select largest cone detection
    int area = 0;
    int largest = 0;
    for (unsigned int i = 0; i < boundBoxes.size(); i++)
    {
        if (boundBoxes[i].width * boundBoxes[i].height > area)
        {
            area = boundBoxes[i].width * boundBoxes[i].height;
            largest = i;
        }
    }

    vector<Point> coneContour = goodContours[largest];
    // Scale the cone contour down
    Moments mnts = moments(coneContour);
    int cx = (int)(mnts.m10 / mnts.m00);
    int cy = (int)(mnts.m01 / mnts.m00);
    for (size_t i = 0; i < coneContour.size(); i++)
    {
        // Center it
        coneContour[i].x -= cx;
        coneContour[i].y -= cy;
        // Scale it
        coneContour[i].x *= 0.8;
        coneContour[i].y *= 0.8;
        // Move it back to original position
        coneContour[i].x += cx;
        coneContour[i].y += cy;
    }

    Rect cone = boundBoxes[largest];

    int coneCenterX = cone.x + (cone.width / 2);
    int coneCenterY = cone.y + (cone.height / 2);

    line(colorFrame, Point(coneCenterX - 5, coneCenterY), Point(coneCenterX + 5, coneCenterY),
         Scalar(255, 255, 255), 1);
    line(colorFrame, Point(coneCenterX, coneCenterY + 5), Point(coneCenterX, coneCenterY - 5),
         Scalar(255, 255, 255), 1);

    Mat depthData(Size(width, height), CV_16UC1, (uint16_t *)depthFrame.get_data());
    double depthUnit = depthFrame.get_units();

    /*
     * Getting depth from correct region:
     * Create a mask from the scaled-down cone contour and a mask from
     * the box at the center of the cone. bitwise_and them together, and
     * use the result as the filter in the mean(), so we only consider
     * depth from pixels that are in the square and within that scaled-down
     * cone
     */
    Mat scaledConeMask = Mat::zeros(Size(width, height), CV_8UC1);
    drawContours(scaledConeMask, vector<vector<Point>>(1, coneContour), -1, 255, -1);

    drawContours(colorFrame, vector<vector<Point>>(1, coneContour), -1, Scalar(0, 0, 255), 1);

    Mat depthRoiMask = Mat::zeros(Size(width, height), CV_8UC1);
    Rect depthRoi(coneCenterX - 10, coneCenterY - 10, 20, 20);
    rectangle(colorFrame, depthRoi, Scalar(100, 100, 100), 1);
    rectangle(depthRoiMask, depthRoi, 255, -1);

    bitwise_and(depthRoiMask, scaledConeMask, depthRoiMask);

    double distAvg = mean(depthData, depthRoiMask)[0] * depthUnit;

    cout << "Distance: " << distAvg << endl;

    double pxFromCenter = (coneCenterX - (width / 2));
    // The 0.108 is a camera property: FOV_in_degrees / frame_width
    double angle = pxFromCenter * 0.108;
    angle *= M_PI / 180;
    double x = distAvg * sin(angle);
    double y = distAvg * cos(angle);
    cout << "X offset: " << x << endl;
    cout << "Y offset: " << y << endl;

    // Example trackbars
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
    cout << "Hue in box: " << hue[0] << endl << "Saturation in box: " << hue[1] << endl << "Value in
    box: " << hue[2] << endl;

    line(colorFrame, Point(width / 2 - box_size, height / 2 - box_size),
         Point(width / 2 + box_size, height / 2 - box_size), Scalar(255, 255, 255), 2);

    line(colorFrame, Point(width / 2 - box_size, height / 2 + box_size),
         Point(width / 2 + box_size, height / 2 + box_size), Scalar(255, 255, 255), 2);

    line(colorFrame, Point(width / 2 - box_size, height / 2 - box_size),
         Point(width / 2 - box_size, height / 2 + box_size), Scalar(255, 255, 255), 2);

    line(colorFrame, Point(width / 2 + box_size, height / 2 - box_size),
         Point(width / 2 + box_size, height / 2 + box_size), Scalar(255, 255, 255), 2);
     */

    // imshow("rgb", colorFrame);
    // imshow("depth", depthData);
    // imshow("hsv", hsvFrame);
    // imshow("hue mask", hueMask);
    // imshow("saturation mask", satMask);
    // imshow("value mask", valMask);
    // imshow("mask", mask);
    // imshow("cone", justCone);
    // imshow("saturation", channels[1]);
    // imshow("value", channels[2]);
    return pair(x, y);
}

double depthCamera::findPoles()
{
    Mat depthData(Size(width, height), CV_16UC1, (uint16_t *)depthFrame.get_data());
    double depthUnit = depthFrame.get_units();

    double min_dist_m = 1.9;
    double max_dist_m = 2;

    int min_dist = min_dist_m / depthUnit;
    int max_dist = max_dist_m / depthUnit;

    Mat mask;
    inRange(depthData, min_dist, max_dist, mask);
    Mat edges;
    // bitwise_and(mask, colorFrame, justPole);

    imshow("pole", mask);
}
