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

#include <opencv2/opencv.hpp>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
#include <apriltag/common/getopt.h>
}

using namespace std;
using namespace cv;

// Camera properties
#define CAM_FX 1656
#define CAM_FY 1656
#define CAM_CX 1199
#define CAM_CY 841

// AprilTag Parameters
#define HAMM_HIST_MAX 10
#define HAMMING_NUMBER 0
#define QUAD_DECIMATE 2.0
#define QUAD_SIGMA 0.0
#define NTHREADS 4
#define APRIL_DEBUG 0
#define REFINE_EDGES 1

int main(int argc, char *argv[])
{
    // Initialize camera
    VideoCapture cap(0);
    if (!cap.isOpened()) {
        cerr << "Couldn't open video capture device" << endl;
        return -1;
    }

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

    Mat frame, gray;
    while (true) {
        cap >> frame;
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header for the Mat data
        image_u8_t im = { .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(td, &im);

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            apriltag_pose_t pose;
            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = 0.01524;
            info.fx = CAM_FX;
            info.fy = CAM_FY;
            info.cx = CAM_CX;
            info.cy = CAM_CY;

            double err = estimate_tag_pose(&info, &pose);

            printf("Rotation\n    %f | %f | %f\n    %f | %f | %f\n    %f | %f | %f\n\n",
                   pose.R->data[0], pose.R->data[1], pose.R->data[2],
                   pose.R->data[3], pose.R->data[4], pose.R->data[5],
                   pose.R->data[6], pose.R->data[7], pose.R->data[8]);
            printf("translation\n X: %f\n Y: %f\n Z: %f\n",
                   pose.t->data[0] * 393.701, pose.t->data[1] * 393.701, pose.t->data[2] * 393.701);

            printf("Err: %f\n\n\n", err);

            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
            line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
            line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
            line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);
            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
            putText(frame, text, Point(det->c[0]-textsize.width/2,
                                       det->c[1]+textsize.height/2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
        }
        apriltag_detections_destroy(detections);

        // Center crosshair
        line(frame, Point(frame.cols / 2 + 10, frame.rows/2),
                 Point(frame.cols/ 2 - 10, frame.rows/2),
                 Scalar(0xff, 0x00, 0xff), 2);
        line(frame, Point(frame.cols/2, frame.rows/2 + 10),
                 Point(frame.cols/2, frame.rows/2 - 10),
                 Scalar(0xff, 0x00, 0xff), 2);

        imshow("Tag Detections", frame);
        if (waitKey(1) >= 0)
            break;
    }

    apriltag_detector_destroy(td);

    tag16h5_destroy(tf);

    return 0;
}
