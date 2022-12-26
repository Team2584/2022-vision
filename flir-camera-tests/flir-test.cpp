/* Copyright (C) 2013-2016, The Regents of The University of Michigan.
All rights reserved.
This software was developed in the APRIL Robotics Lab under the
direction of Edwin Olson, ebolson@umich.edu. This software may be
available under alternative licensing terms; contact the address above.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the Regents of The University of Michigan.
*/

#include <iostream>
#include <iomanip>
#include <sstream>
#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <opencv2/opencv.hpp>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/tag16h5.h>
#include <apriltag/apriltag_pose.h>
}


// AprilTag Parameters
#define HAMM_HIST_MAX 10
#define HAMMING_NUMBER 0
#define QUAD_DECIMATE 2.0
#define QUAD_SIGMA 0.0
#define NTHREADS 4
#define APRIL_DEBUG 0
#define REFINE_EDGES 1

using namespace std;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace cv;

int main(int argc, char *argv[])
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
    INodeMap& nodeMap = pCam1->GetNodeMap();

    // Recieve enumeration node from nodemap
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
        cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
        return -1;
    }

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
        cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..." << endl << endl;
        return -1;
    }

    // Retrieve integer value from entry node
    const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    // Retrieve Stream Parameters device nodemap
    Spinnaker::GenApi::INodeMap& sNodeMap = pCam1->GetTLStreamNodeMap();

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
    //Select blue channel balance ratio
    // pCam1->BalanceRatioSelector.SetValue(BalanceRatioSelector_Blue);
    //Set the white balance blue channel to 2
    // CFloatPtr BalanceRatio = nodeMap.GetNode("BalanceRatio");
    // BalanceRatio->SetValue(2);
    //Set the white balance red channel to 2
    // pCam1->BalanceRatioSelector.SetValue(BalanceRatioSelector_Red);
    // BalanceRatio->SetValue(2);

    pCam1->BeginAcquisition();

    /**********************************************************************************************
     * AprilTags Setup *
     *******************/

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    tf = tag16h5_create();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family_bits(td, tf, HAMMING_NUMBER);

    switch(errno){
        case EINVAL:
            printf("\"hamming\" parameter is out-of-range.\n");
            exit(-1);
        case ENOMEM:
            printf("Unable to add family to detector due to insufficient memory to allocate the tag-family decoder. Try reducing \"hamming\" from %d or choose an alternative tag family.\n", HAMMING_NUMBER);
            exit(-1);
    }

    td->quad_decimate = QUAD_DECIMATE;
    td->quad_sigma = QUAD_SIGMA;
    td->nthreads = NTHREADS;
    td->debug = APRIL_DEBUG;
    td->refine_edges = REFINE_EDGES;

    int total_hamm_hist[HAMM_HIST_MAX];
    memset(total_hamm_hist, 0, sizeof(int)*HAMM_HIST_MAX);



    /**********************************************************************************************
     * THE LOOP *
     ************/
    ImagePtr frame = nullptr;
    ImageStatus framestatus;

    Mat matframe(540, 720, CV_8UC1);

    while (true) {
        errno = 0;
        // Grab frame and check its status
        frame = pCam1->GetNextImage();
        framestatus = frame->GetImageStatus();
        if (framestatus != 0)
            printf("FRAME ERROR");

        // The Mat business is only for displaying results
        matframe.data = (uint8_t *)frame->GetData();

        int hamm_hist[HAMM_HIST_MAX];
        memset(hamm_hist, 0, sizeof(hamm_hist));

        // Make an image_u8_t header from the frame
        image_u8_t im = {
            .width = (int) frame->GetWidth(),
            .height = (int) frame->GetHeight(),
            .stride = (int) frame->GetStride(),
            .buf = (uint8_t *)frame->GetData(),
        };

        // Do the detecting
        zarray_t *detections = apriltag_detector_detect(td, &im);

        if (errno == EAGAIN) {
            printf("Unable to create the %d threads requested.\n",td->nthreads);
            exit(-1);
        }

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            printf("DETECT ");
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            apriltag_pose_t pose;
            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = 0.017;
            info.fx = 720 / 4.98;
            info.fy = 540 / 3.74;
            info.cx = 720 / 2;
            info.cy = 540 / 2;

            double err = estimate_tag_pose(&info, &pose);

            line(matframe, Point(det->p[0][0], det->p[0][1]),
                 Point(det->p[1][0], det->p[1][1]),
                 Scalar(0xff, 0xff, 0xff), 2);
            line(matframe, Point(det->p[0][0], det->p[0][1]),
                 Point(det->p[3][0], det->p[3][1]),
                 Scalar(0xff, 0xff, 0xff), 2);
            line(matframe, Point(det->p[1][0], det->p[1][1]),
                 Point(det->p[2][0], det->p[2][1]),
                 Scalar(0xff, 0xff, 0xff), 2);
            line(matframe, Point(det->p[2][0], det->p[2][1]),
                 Point(det->p[3][0], det->p[3][1]),
                 Scalar(0xff, 0xff, 0xff), 2);

            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                            &baseline);
            putText(matframe, text, Point(det->c[0]-textsize.width/2,
                    det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
            hamm_hist[det->hamming]++;
            total_hamm_hist[det->hamming]++;
        }
        printf("-\n");

        apriltag_detections_destroy(detections);

        imshow("Tag Detections", matframe);

        if (waitKey(1) == 'q')
            break;
    }

    apriltag_detector_destroy(td);
    tag16h5_destroy(tf);

    pCam1 = nullptr;

    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    return 0;
}
