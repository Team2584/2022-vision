#include <iomanip>
#include <iostream>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdio.h>
#include <vector>

using namespace cv;
using namespace std;

int CHECKERBOARD[2]{9, 6};

// Edge length of one square in mm
#define SQUARE_SIZE 25

int main()
{
    /**********************************************************************************************
     * Camera SETUP *
     *************/
    rs2::config cfg;
    rs2::pipeline pipe;

    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);

    // Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    /****************************************************************
     * Other Things
     **************/

    vector<vector<Point3f>> objpoints;
    vector<vector<Point2f>> imgpoints;

    vector<Point3f> objp;
    for (int i = 0; i < CHECKERBOARD[1]; i++)
    {
        for (int j = 0; j < CHECKERBOARD[0]; j++)
        {
            objp.push_back(Point3f(j * SQUARE_SIZE, i * SQUARE_SIZE, 0));
        }
    }

    Mat matframe(cv::Size(640, 480), CV_8UC3);
    Mat saveframe(cv::Size(640, 480), CV_8UC3);
    vector<Mat> images;

    rs2::frameset frames;
    while (true)
    {
        // Grab frame and check its status
        frames = pipe.wait_for_frames();

        rs2::video_frame frame = frames.get_color_frame();

        matframe.data = (uint8_t *)frame.get_data();

        // The Mat business is only for displaying results
        char key = waitKey(1);
        imshow("Camera Feed", matframe);
        if (key == ' ')
        {
            saveframe = matframe.clone();
            images.push_back(saveframe);
            printf("Image added to list\n");
        }
        else if (key == 'q')
        {
            break;
        }
    }

    destroyAllWindows();

    vector<Point2f> corner_pts;
    bool success;
    Mat current(cv::Size(640, 480), CV_8UC3);
    Mat gray(cv::Size(640, 480), CV_8UC1);
    printf("\nBeginning processing\n");

    for (int i = 0; i < images.size(); i++)
    {
        current = images[i];
        cv::cvtColor(current, gray, COLOR_BGR2GRAY);
        success = findChessboardCorners(gray, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
                                        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK |
                                            CALIB_CB_NORMALIZE_IMAGE);

        printf("Processing... ");

        if (success)
        {
            printf("Image valid.\n");
            TermCriteria criteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001);
            cornerSubPix(gray, corner_pts, Size(11, 11), Size(-1, -1), criteria);
            drawChessboardCorners(current, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
                                  success);
            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }
        else
        {
            printf("Grid not found. Discarding image.\n");
        }
        imshow("Image", current);
        waitKey(1000);
    }

    destroyAllWindows();
    printf("\nBeginning calibration.\n");
    Mat cameraMatrix, distCoeffs, R, T;
    calibrateCamera(objpoints, imgpoints, Size(matframe.rows, matframe.cols), cameraMatrix,
                    distCoeffs, R, T);
    cout << "Camera Matrix: " << cameraMatrix << endl;
    cout << "Distortion Coefficients: " << distCoeffs << endl;
}
