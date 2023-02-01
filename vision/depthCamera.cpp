#include "Cameras.h"
#include <chrono>

using namespace std;
using namespace cv;

int epsCoeffPercent = 3;
int cksize = 30;
int oksize = 6;

depthCamera::depthCamera(string camSerial, int width, int height, int fps)
    : pipe{}, cfg{}, prof{}, align{RS2_STREAM_COLOR}, colorFrame{cv::Size(width, height), CV_8UC3},
      grayFrame{cv::Size(width, height), CV_8UC1}, depthFrame{nullptr}
{
    this->width = width;
    this->height = height;

    // Select camera by serial number
    cfg.enable_device(camSerial);

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);

    // Instruct pipeline to start streaming with the requested configuration
    prof = pipe.start(cfg);

    // Disgusting one-liner to turn on/off laser
    prof.get_device().first<rs2::depth_sensor>().set_option(RS2_OPTION_EMITTER_ENABLED, 1.f);
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

    auto start = chrono::steady_clock::now();
    frames = align.process(frames);
    auto end = chrono::steady_clock::now();
    auto dur = chrono::duration_cast<chrono::milliseconds>(end - start);
    cout << "Time for aligning: " << dur.count() << "ms" << endl;

    rs2::video_frame frame = frames.get_color_frame();
    depthFrame = frames.get_depth_frame();

    colorFrame.data = (uint8_t *)frame.get_data();
    cv::cvtColor(colorFrame, grayFrame, cv::COLOR_BGR2GRAY);
}

std::pair<double, double> depthCamera::findCones()
{
    Mat hsvFrame;
    GaussianBlur(colorFrame, hsvFrame, Size(17, 17), 1.2, 1.2, BORDER_DEFAULT);
    cvtColor(colorFrame, hsvFrame, COLOR_BGR2HSV);

    uint8_t min_hue = 8;
    uint8_t max_hue = 30;
    uint8_t min_sat = 200;
    uint8_t max_sat = 255;
    uint8_t min_val = 120;
    uint8_t max_val = 255;

    Mat mask;
    inRange(hsvFrame, Scalar(min_hue, min_sat, min_val), Scalar(max_hue, max_sat, max_val), mask);

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

    Mat coneMask = Mat::zeros(Size(width, height), CV_8UC1);
    Mat depthRoi(depthData, cone);
    Mat depthRoiView;
    depthRoi.convertTo(depthRoiView, CV_8UC1);
    // imshow("things", depthRoiView);
    drawContours(coneMask, vector<vector<Point>>(1, coneContour), -1, 255, -1);

    double distAvg = mean(depthData, coneMask)[0] * depthUnit;
    double centerDist = depthFrame.get_distance(coneCenterX, coneCenterY);

    cout << "Distance: " << distAvg * INCH << endl;
    cout << "Other Distance: " << centerDist * INCH << endl;

    Mat depthView;
    depthData.convertTo(depthView, CV_8UC1);
    line(colorFrame, Point((width / 2) - 5, (height / 2)), Point((width / 2) + 5, (height / 2)),
         Scalar(255, 255, 255), 1);
    line(colorFrame, Point((width / 2), (height / 2) + 5), Point((width / 2), (height / 2) - 5),
         Scalar(255, 255, 255), 1);

    line(depthView, Point((width / 2) - 5, (height / 2)), Point((width / 2) + 5, (height / 2)),
         Scalar(255, 255, 255), 1);
    line(depthView, Point((width / 2), (height / 2) + 5), Point((width / 2), (height / 2) - 5),
         Scalar(255, 255, 255), 1);

    // imshow("depth", depthView / 2);
    double XpxFromCenter = (coneCenterX - (width / 2));
    double YpxFromCenter = ((height / 2) - coneCenterY);
    // The 0.108 is a camera property: FOV_in_degrees / frame_width
    // Same goes for the vertical one
    double xAngle = XpxFromCenter * 0.108;
    double yAngle = YpxFromCenter * 0.089;
    cout << "angle: " << yAngle << endl;
    xAngle *= M_PI / 180;
    yAngle *= M_PI / 180;
    // Convert from spherical coordinates to x, y, z
    double x = distAvg * cos(yAngle) * sin(xAngle);
    double y = distAvg * cos(yAngle) * cos(xAngle);
    double z = distAvg * sin(yAngle);

    // Compensate for the camera being tilted down by rotating the vector
    Eigen::Vector3d pos;
    pos << x, y, z;

    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << 1, 0, 0, 0, cos(info.offset.elevAngle), -sin(info.offset.elevAngle), 0,
        sin(info.offset.elevAngle), cos(info.offset.elevAngle);

    pos = rotationMatrix * pos;

    x = pos(0) + info.offset.x;
    y = pos(1) + info.offset.y;
    z = pos(2) + info.offset.z;

    cout << "Cone X: " << x << endl;
    cout << "Cone Y: " << y << endl;
    cout << "Cone Z: " << z << endl;

    return pair(x, y);
}

std::pair<double, double> depthCamera::findCubes()
{
    Mat hsvFrame;
    GaussianBlur(colorFrame, hsvFrame, Size(17, 17), 1.2, 1.2, BORDER_DEFAULT);

    cvtColor(colorFrame, hsvFrame, COLOR_BGR2HSV);

    uint8_t min_hue = 94;
    uint8_t max_hue = 133;
    uint8_t min_sat = 119;
    uint8_t max_sat = 212;
    uint8_t min_val = 10;
    uint8_t max_val = 189;

    Mat mask;
    inRange(hsvFrame, Scalar(min_hue, min_sat, min_val), Scalar(max_hue, max_sat, max_val), mask);

    // Morphological open to get rid of nonsense
    Mat morphedMask;
    createTrackbar("open kernel size", "depth_red", &oksize, 50);
    createTrackbar("close kernel size", "depth_red", &cksize, 50);
    Mat openKernel = getStructuringElement(MORPH_RECT, Size(oksize, oksize));
    morphologyEx(mask, morphedMask, MORPH_OPEN, openKernel);

    Mat closeKernel = getStructuringElement(MORPH_RECT, Size(cksize, cksize));
    morphologyEx(morphedMask, morphedMask, MORPH_CLOSE, closeKernel);

    // imshow("mask", morphedMask);

    // Get contours of cones
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(morphedMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // Show contours & bounding boxes
    vector<Rect> boundBoxes;
    vector<vector<Point>> goodContours;
    for (unsigned int i = 0; i < contours.size(); i++)
    {
        Rect boundBox = boundingRect(contours[i]);
        // if (boundBox.width < 10 || boundBox.height < 10) // Discard small detections
        // continue;

        goodContours.push_back(contours[i]);
        boundBoxes.push_back(boundBox);
        drawContours(colorFrame, contours, i, Scalar(0, 255, 0), 2, FILLED);
        rectangle(colorFrame, boundBox, Scalar(255, 0, 0));
    }

    // Get convex hulls of contours
    vector<vector<Point>> hulls(goodContours.size());
    for (uint16_t i = 0; i < goodContours.size(); i++)
    {
        convexHull(contours[i], hulls[i]);
        // drawContours(colorFrame, hulls, i, Scalar(0, 0, 255), 2, FILLED);
    }

    // Approximate contours
    createTrackbar("epsilon coefficient", "depth_red", &epsCoeffPercent, 100);
    vector<vector<Point>> approx(contours.size());
    double epsCoeff = epsCoeffPercent / 100.0;
    for (uint16_t i = 0; i < contours.size(); i++)
    {
        double eps = epsCoeff * arcLength(contours[i], true);
        approxPolyDP(hulls[i], approx[i], eps, true);
        if (approx[i].size() <= 6)
            drawContours(colorFrame, approx, i, Scalar(0, 0, 255), 2, FILLED);
    }

    // If there are no cones
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

    vector<Point> cubeContour = approx[largest];
    Rect cube = boundBoxes[largest];

    int cubeCenterX = cube.x + (cube.width / 2);
    int cubeCenterY = cube.y + (cube.height / 2);

    line(colorFrame, Point(cubeCenterX - 5, cubeCenterY), Point(cubeCenterX + 5, cubeCenterY),
         Scalar(255, 255, 255), 1);
    line(colorFrame, Point(cubeCenterX, cubeCenterY + 5), Point(cubeCenterX, cubeCenterY - 5),
         Scalar(255, 255, 255), 1);

    Mat depthData(Size(width, height), CV_16UC1, (uint16_t *)depthFrame.get_data());
    double depthUnit = depthFrame.get_units();

    Mat depthRoi(depthData, cube);
    Mat cubeMask = Mat::zeros(Size(width, height), CV_8UC1);
    drawContours(cubeMask, vector<vector<Point>>(1, cubeContour), -1, 255, -1);

    double distAvg = mean(depthData, cubeMask)[0] * depthUnit;
    double centerDist = depthFrame.get_distance(cubeCenterX, cubeCenterY);

    cout << "Distance: " << distAvg * INCH << endl;
    cout << "Other Distance: " << centerDist * INCH << endl;

    double XpxFromCenter = (cubeCenterX - (width / 2));
    double YpxFromCenter = ((height / 2) - cubeCenterY);
    // The 0.108 is a camera property: FOV_in_degrees / frame_width
    // Same goes for the vertical one
    double xAngle = XpxFromCenter * 0.108;
    double yAngle = YpxFromCenter * 0.089;
    cout << "angle: " << yAngle << endl;
    xAngle *= M_PI / 180;
    yAngle *= M_PI / 180;
    // Convert from spherical coordinates to x, y, z
    double x = distAvg * cos(yAngle) * sin(xAngle);
    double y = distAvg * cos(yAngle) * cos(xAngle);
    double z = distAvg * sin(yAngle);

    // Compensate for the camera being tilted down by rotating the vector
    Eigen::Vector3d pos;
    pos << x, y, z;

    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << 1, 0, 0, 0, cos(info.offset.elevAngle), -sin(info.offset.elevAngle), 0,
        sin(info.offset.elevAngle), cos(info.offset.elevAngle);

    pos = rotationMatrix * pos;

    x = pos(0) + info.offset.x;
    y = pos(1) + info.offset.y;
    z = pos(2) + info.offset.z;

    cout << "Cube X: " << x << endl;
    cout << "Cube Y: " << y << endl;
    cout << "Cube Z: " << z << endl;

    return pair(x, y);
}

std::pair<double, double> depthCamera::findPoles()
{
    /*
    Mat depthData(Size(width, height), CV_16UC1, (uint16_t *)depthFrame.get_data());
    double depthUnit = depthFrame.get_units();

    Mat depthEdges;
    Canny(depthData, edges, CV_64F, 1, 1);

    Mat edgesView;
    edges.convertTo(edgesView, CV_8UC1);

    imshow("depth edges", edges);
    */
}
