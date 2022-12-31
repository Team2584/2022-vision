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

#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>

#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/BooleanTopic.h>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
}

// AprilTag Parameters
#define HAMM_HIST_MAX 10
#define HAMMING_NUMBER 0
#define QUAD_DECIMATE 2.0
#define QUAD_SIGMA 0.0
#define NTHREADS 4
#define APRIL_DEBUG 0
#define REFINE_EDGES 1

// Camera parameters
#define CAM_CX 323
#define CAM_CY 245
#define CAM_FX 608
#define CAM_FY 608

using namespace std;
using namespace cv;
using namespace Eigen;

int main()
{
    /**********************************************************************************************
     * DEPTH CAMPERA SETUP *
     ***********************/

    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_ANY, 30);

    //Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    /**********************************************************************************************
     * AprilTags Setup *
     *******************/

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    tf = tag16h5_create();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family_bits(td, tf, HAMMING_NUMBER);


    td->quad_decimate = QUAD_DECIMATE;
    td->quad_sigma = QUAD_SIGMA;
    td->nthreads = NTHREADS;
    td->debug = APRIL_DEBUG;
    td->refine_edges = REFINE_EDGES;

    int total_hamm_hist[HAMM_HIST_MAX];
    memset(total_hamm_hist, 0, sizeof(int)*HAMM_HIST_MAX);

    /**********************************************************************************************
     * Network Tables Setup *
     ************************/

    // Create networktables instan8ce and a table for vision
    nt::NetworkTableInstance nt_inst= nt::NetworkTableInstance::GetDefault();

    // Setup networktable client
    nt_inst.StartClient4("jetson client");
    nt_inst.SetServerTeam(2584);
    nt_inst.StartDSClient();
    nt_inst.SetServer("host", NT_DEFAULT_PORT4);

    // Create tables
    shared_ptr<nt::NetworkTable> visionTbl = nt_inst.GetTable("vision");
    shared_ptr<nt::NetworkTable> localTbl = nt_inst.GetTable("vision/localization");

    // Make a sanity check topic and an entry to publish/read from it; set initial value
    nt::StringTopic sanitycheck = visionTbl->GetStringTopic("sanitycheck");
    nt::StringEntry sanitycheckEntry = sanitycheck.GetEntry("DEFAULT SANITYCHECK SET BY JETSON");
    sanitycheckEntry.Set("Hello, world. I've been set up!");

    // Other vision topics
    nt::BooleanTopic robot_pos_goodTopic = localTbl->GetBooleanTopic("robot_pos_good");
    nt::DoubleTopic robot_xTopic = localTbl->GetDoubleTopic("x");
    nt::DoubleTopic robot_yTopic = localTbl->GetDoubleTopic("y");
    nt::DoubleTopic robot_thetaTopic = localTbl->GetDoubleTopic("theta");

    nt::BooleanEntry robot_pos_goodEntry = robot_pos_goodTopic.GetEntry(false);
    nt::DoubleEntry robot_xEntry = robot_xTopic.GetEntry(0.0);
    nt::DoubleEntry robot_yEntry = robot_yTopic.GetEntry(0.0);
    nt::DoubleEntry robot_thetaEntry = robot_thetaTopic.GetEntry(0.0);

    /**********************************************************************************************
     * THE LOOP *
     ************/

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;

    for(int i = 0; i < 30; i++) { frames = pipe.wait_for_frames(); }

    Mat matframe(Size(640, 480), CV_8UC3, (uint8_t *) frames.get_color_frame().get_data(), Mat::AUTO_STEP);
    Mat gray(Size(640, 480), CV_8UC1);

    Matrix3f poseRotationMatrix;
    Vector3f poseAngles; 

    // Rotation of tag relative to camera
    double rotX;
    double rotY;
    double rotZ;

    // Translation of tag relative to camera
    double linX;
    double linY;
    double linZ;

    double poseErr;

    while (true) {
        errno = 0;

        // Make sure networktables is working
        sanitycheckEntry.Set("We have entered the loop (on the Jetson).");
        
        // Grab a frame
        frames = pipe.wait_for_frames();
        rs2::video_frame frame = frames.get_color_frame();

        // The Mat business is only needed for displaying results
        matframe.data = (uint8_t *)frame.get_data();
        cvtColor(matframe, gray, COLOR_BGR2GRAY);

        int hamm_hist[HAMM_HIST_MAX];
        memset(hamm_hist, 0, sizeof(hamm_hist));

        // Make an image_u8_t header from the frame
        image_u8_t im = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data,
        };

        // Do the detecting
        zarray_t *detections = apriltag_detector_detect(td, &im);

        if (errno == EAGAIN) {
            printf("Unable to create the %d threads requested.\n",td->nthreads);
            exit(-1);
        }

        // Tag doesn't exist
        robot_pos_goodEntry.Set(false);

        // Loop through detections
        for (int i = 0; i < zarray_size(detections); i++) {
            // Get the detection
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            // Only use valid tag detections
            if (det->id != 0)
                continue;

            // Tag found
            robot_pos_goodEntry.Set(true);

            // Apriltag pose estimation
            apriltag_pose_t pose;
            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = 0.01524;
            info.fx = CAM_FX;
            info.fy = CAM_FY;
            info.cx = CAM_CX;
            info.cy = CAM_CY;

            poseErr = estimate_tag_pose(&info, &pose);

            // Calculate pose angles based on rotation matrix
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    poseRotationMatrix(i, j) = pose.R->data[3 * i + j];
            poseAngles = poseRotationMatrix.eulerAngles(0, 1, 2);

            // All rotations are *around* the axes they're named based on
            rotX = poseAngles(0); // Left/right rel. to camera
            rotZ = poseAngles(1); // Up/down rel. to camera
            rotY = poseAngles(2); // Out/in rel. to camera

            // Tranlation in meters
            linX = pose.t->data[0] * 10;
            linY = pose.t->data[2] * 10;
            linZ = pose.t->data[1] * 10;

            // Convert to degrees
            /*
            rotX *= (180 / M_PI);
            rotY *= (180 / M_PI);
            rotZ *= (180 / M_PI);
            */
            rotZ += M_PI;

            if (rotZ < 50 || rotZ > 310)
            {
                // Send relevant info to networkTables (it's negative so it's relative to tag)
                robot_xEntry.Set(-linX);
                robot_yEntry.Set(-linY);
                robot_thetaEntry.Set(rotZ);

                // Print info
                printf("Translation\n X: %f\n Y: %f\n Z: %f\n", linX, linY, linZ);
                printf("Rotation\n Around X: %f\n Around Y: %f\n Around Z: %f\n", rotX, rotY, rotZ);

                printf("Pose Error: %f\n\n\n", poseErr);

                // Rotation matrix
                printf("Rotation\n    %f | %f | %f\n    %f | %f | %f\n    %f | %f | %f\n\n",
                       pose.R->data[0], pose.R->data[1], pose.R->data[2],
                       pose.R->data[3], pose.R->data[4], pose.R->data[5],
                       pose.R->data[6], pose.R->data[7], pose.R->data[8]);
            }

            hamm_hist[det->hamming]++;
            total_hamm_hist[det->hamming]++;

            // Draw detection outline
            line(matframe, Point(det->p[0][0], det->p[0][1]),
                 Point(det->p[1][0], det->p[1][1]),
                 Scalar(0xff, 0x00, 0x00), 2);
            line(matframe, Point(det->p[0][0], det->p[0][1]),
                 Point(det->p[3][0], det->p[3][1]),
                 Scalar(0x00, 0xff, 0x00), 2);
            line(matframe, Point(det->p[1][0], det->p[1][1]),
                 Point(det->p[2][0], det->p[2][1]),
                 Scalar(0x00, 0x00, 0xff), 2);
            line(matframe, Point(det->p[2][0], det->p[2][1]),
                 Point(det->p[3][0], det->p[3][1]),
                 Scalar(0xff, 0x00, 0xff), 2);

            // Label the tag
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
        }

        apriltag_detections_destroy(detections);

        imshow("Tag Detections", matframe);

        if (waitKey(1) == 'q')
            break;
    }

    apriltag_detector_destroy(td);
    tag16h5_destroy(tf);

    return 0;
}
