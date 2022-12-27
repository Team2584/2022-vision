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
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

extern "C" {
#include <apriltag/apriltag.h>
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

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
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
     * THE LOOP *
     ************/
    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;

    for(int i = 0; i < 30; i++) { frames = pipe.wait_for_frames(); }

    Mat matframe(Size(640, 480), CV_8UC3, (uint8_t *) frames.get_color_frame().get_data(), Mat::AUTO_STEP);
    Mat gray(Size(640, 480), CV_8UC1);

    while (true) {
        errno = 0;
        
        // Grab a frame
        frames = pipe.wait_for_frames();
        rs2::video_frame frame = frames.get_color_frame();

        // The Mat business is only for displaying results
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

        // Draw detection outlines
        for (int i = 0; i < zarray_size(detections); i++) {
            printf("DETECT ");
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

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

    return 0;
}
