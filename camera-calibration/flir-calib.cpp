#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <stdio.h>
#include <vector>

using namespace cv;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

int CHECKERBOARD[2]{9, 6};

// Edge length of one square in mm
#define SQUARE_SIZE 25

int main()
{
    /**********************************************************************************************
     * FLIR SETUP *
     *************/

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    const unsigned int numCameras = camList.GetSize();

    // cout << "Number of cameras detected: " << numCameras << endl << endl;

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        camList.Clear();
        system->ReleaseInstance();
        cout << "Not enough cameras!" << endl;
        return -1;
    }

    // Select camera
    CameraPtr pCam1 = nullptr;
    pCam1 = camList.GetByIndex(0);
    pCam1->Init();

    // Retrieve GenICam nodemap
    INodeMap &nodeMap = pCam1->GetNodeMap();

    // Recieve enumeration node from nodemap
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
        cout << "Unable to set acquisition mode to continuous (enum retrieval). "
                "Aborting..."
             << endl
             << endl;
        return -1;
    }

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
        cout << "Unable to set acquisition mode to continuous (entry retrieval). "
                "Aborting..."
             << endl
             << endl;
        return -1;
    }

    // Retrieve integer value from entry node
    const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    // Retrieve Stream Parameters device nodemap
    Spinnaker::GenApi::INodeMap &sNodeMap = pCam1->GetTLStreamNodeMap();

    // Retrieve Buffer Handling Mode Information
    CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
    if (!IsAvailable(ptrHandlingMode) || !IsWritable(ptrHandlingMode))
    {
        cout << "Unable to set Buffer Handling mode (node retrieval). Aborting..." << endl << endl;
        return -1;
    }

    CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetCurrentEntry();
    if (!IsAvailable(ptrHandlingModeEntry) || !IsReadable(ptrHandlingModeEntry))
    {
        cout << "Unable to set Buffer Handling mode (Entry retrieval). Aborting..." << endl << endl;
        return -1;
    }

    // Set buffer handling mode
    ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestOnly");
    ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());

    // Exposure
    pCam1->ExposureAuto.SetValue(ExposureAuto_Continuous);
    // pCam1->ExposureAuto.SetValue(ExposureAuto_Off);
    // pCam1->ExposureMode.SetValue(ExposureMode_Timed);
    // pCam1->ExposureTime.SetValue(20000);

    // Gain
    pCam1->GainAuto.SetValue(GainAutoEnums::GainAuto_Once);
    // pCam1->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Off);
    // pCam1->Gain.SetValue(10.5);

    // Gamma
    pCam1->Gamma.SetValue(1.0);

    // White Balance
    // pCam1->BalanceWhiteAuto.SetValue(BalanceWhiteAutoEnums::BalanceWhiteAuto_Once);
    // pCam1->BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Off);
    // Select blue channel balance ratio
    // pCam1->BalanceRatioSelector.SetValue(BalanceRatioSelector_Blue);
    // Set the white balance blue channel to 2
    // CFloatPtr BalanceRatio = nodeMap.GetNode("BalanceRatio");
    // BalanceRatio->SetValue(2);
    // Set the white balance red channel to 2
    // pCam1->BalanceRatioSelector.SetValue(BalanceRatioSelector_Red);
    // BalanceRatio->SetValue(2);

    pCam1->BeginAcquisition();

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

    ImagePtr frame = nullptr;
    ImageStatus framestatus;
    frame = pCam1->GetNextImage();
    ImagePtr cpframe(frame);
    Mat matframe(cv::Size(720, 540), CV_8UC1);
    Mat saveframe(cv::Size(720, 540), CV_8UC1);
    vector<Mat> images;

    while (true)
    {
        // Grab frame and check its status
        frame = pCam1->GetNextImage();
        framestatus = frame->GetImageStatus();
        if (framestatus != 0)
            printf("FRAME ERROR");

        // The Mat business is only for displaying results
        matframe.data = (uint8_t *)frame->GetData();
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

    pCam1 = nullptr;
    // Clear camera list before releasing system
    camList.Clear();
    // Release system
    system->ReleaseInstance();

    destroyAllWindows();

    vector<Point2f> corner_pts;
    bool success;
    Mat displayimg(cv::Size(720, 540), CV_8UC1);
    Mat current(cv::Size(720, 540), CV_8UC1);
    printf("\nBeginning processing\n");

    for (int i = 0; i < images.size(); i++)
    {
        current = images[i];
        cvtColor(current, displayimg, COLOR_GRAY2BGR);
        success = findChessboardCorners(current, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
                                        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK |
                                            CALIB_CB_NORMALIZE_IMAGE);

        printf("Processing... ");

        if (success)
        {
            printf("Image valid.\n");
            TermCriteria criteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001);
            cornerSubPix(current, corner_pts, Size(11, 11), Size(-1, -1), criteria);
            drawChessboardCorners(displayimg, Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts,
                                  success);
            objpoints.push_back(objp);
            imgpoints.push_back(corner_pts);
        }
        else
        {
            printf("Grid not found. Discarding image.\n");
        }
        imshow("Image", displayimg);
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
