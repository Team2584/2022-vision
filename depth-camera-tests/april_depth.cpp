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
#include <iomanip>
#include <iostream>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>
#include <sstream>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
}

// Constants
#define M_TWOPI 2 * M_PI

// AprilTag Parameters
#define HAMM_HIST_MAX 10
#define HAMMING_NUMBER 0
#define QUAD_DECIMATE 2.0
#define QUAD_SIGMA 0.0
#define NTHREADS 4
#define APRIL_DEBUG 0
#define REFINE_EDGES 1

// Camera parameters
#define DEPTH_WIDTH 640
#define DEPTH_HEIGHT 480

#define CAM_CX 323
#define CAM_CY 245
#define CAM_FX 608
#define CAM_FY 608

#define TAG_SIZE 0.01524

#define IMG_MARGIN 20

using namespace std;
using namespace cv;
using namespace Eigen;

// Rotation of tag relative to camera
double rotX;
double rotY;
double rotZ;
Eigen::Vector3d tag_trans;
Eigen::Matrix3d tag_rot;

// Normalize angle to be within the interval [-pi,pi].
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t + M_PI, M_TWOPI) - M_PI;
  } else {
    t = fmod(t - M_PI, -M_TWOPI) + M_PI;
  }
  return t;
}

bool in_margin(double p[]) {
  if (p[0] < IMG_MARGIN || p[0] > DEPTH_WIDTH - IMG_MARGIN)
    return true;
  if (p[1] < IMG_MARGIN || p[1] > DEPTH_HEIGHT - IMG_MARGIN)
    return true;
  return false;
}

// Convert rotation matrix to Euler angles
void wRo_to_euler(const Eigen::Matrix3d &wRo, double &yaw, double &pitch,
                  double &roll) {
  yaw = standardRad(atan2(wRo(1, 0), wRo(0, 0)));
  double c = cos(yaw);
  double s = sin(yaw);
  pitch = standardRad(atan2(-wRo(2, 0), wRo(0, 0) * c + wRo(1, 0) * s));
  roll = standardRad(
      atan2(wRo(0, 2) * s - wRo(1, 2) * c, -wRo(0, 1) * s + wRo(1, 1) * c));
}

Eigen::Matrix4d getRelativeTransform(apriltag_detection_t *det, double tag_size,
                                     double fx, double fy, double px,
                                     double py) {
  std::vector<cv::Point3f> objPts;
  std::vector<cv::Point2f> imgPts;
  double s = tag_size / 2.;
  objPts.push_back(cv::Point3f(-s, -s, 0));
  objPts.push_back(cv::Point3f(s, -s, 0));
  objPts.push_back(cv::Point3f(s, s, 0));
  objPts.push_back(cv::Point3f(-s, s, 0));

  std::pair<float, float> p1(det->p[0][0], det->p[0][1]);
  std::pair<float, float> p2(det->p[1][0], det->p[1][1]);
  std::pair<float, float> p3(det->p[2][0], det->p[2][1]);
  std::pair<float, float> p4(det->p[3][0], det->p[3][1]);
  imgPts.push_back(cv::Point2f(p1.first, p1.second));
  imgPts.push_back(cv::Point2f(p2.first, p2.second));
  imgPts.push_back(cv::Point2f(p3.first, p3.second));
  imgPts.push_back(cv::Point2f(p4.first, p4.second));

  cv::Mat rvec, tvec;
  cv::Matx33d cameraMatrix(fx, 0, px, 0, fy, py, 0, 0, 1);
  cv::Vec4f distParam(0, 0, 0, 0); // all 0?
  cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
  cv::Matx33d r;
  cv::Rodrigues(rvec, r);
  Eigen::Matrix3d wRo;
  wRo << r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1),
      r(2, 2);

  Eigen::Matrix4d T;
  T.topLeftCorner(3, 3) = wRo;
  T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1),
      tvec.at<double>(2);
  T.row(3) << 0, 0, 0, 1;

  return T;
}

void getRelativeTranslationRotation(apriltag_detection_t *det, double tag_size,
                                    double fx, double fy, double px, double py,
                                    Eigen::Vector3d &trans,
                                    Eigen::Matrix3d &rot) {
  Eigen::Matrix4d T = getRelativeTransform(det, tag_size, fx, fy, px, py);

  // converting from camera frame (z forward, x right, y down) to
  // object frame (x forward, y left, z up)
  Eigen::Matrix4d M;
  M << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;
  Eigen::Matrix4d MT = M * T;
  // translation vector from camera to the April tag
  trans = MT.col(3).head(3);
  // orientation of April tag with respect to camera: the camera
  // convention makes more sense here, because yaw,pitch,roll then
  // naturally agree with the orientation of the object
  rot = T.block(0, 0, 3, 3);
}

void getRealTranslationRotation() {}

int main() {
  /**********************************************************************************************
   * DEPTH CAMPERA SETUP *
   ***********************/

  // Contruct a pipeline which abstracts the device
  rs2::pipeline pipe;

  // Create a configuration for configuring the pipeline with a non default
  // profile
  rs2::config cfg;

  // Add desired streams to configuration
  cfg.enable_stream(RS2_STREAM_COLOR, DEPTH_WIDTH, DEPTH_HEIGHT,
                    RS2_FORMAT_BGR8, 60);
  cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_ANY,
                    60);

  // Instruct pipeline to start streaming with the requested configuration
  pipe.start(cfg);

  /**********************************************************************************************
   * LOGITECH CAMERA SETUP *
   ************************/

  VideoCapture cap(0);
  if (cap.isOpened()) {
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(CAP_PROP_FPS, 30);
  }

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
  memset(total_hamm_hist, 0, sizeof(int) * HAMM_HIST_MAX);

  /**********************************************************************************************
   * Network Tables Setup *
   ************************/

  // Create networktables instan8ce and a table for vision
  nt::NetworkTableInstance nt_inst = nt::NetworkTableInstance::GetDefault();

  // Setup networktable client
  nt_inst.StartClient4("jetson client");
  nt_inst.SetServerTeam(2584);
  nt_inst.StartDSClient();
  nt_inst.SetServer("host", NT_DEFAULT_PORT4);

  // Create tables
  shared_ptr<nt::NetworkTable> visionTbl = nt_inst.GetTable("vision");
  shared_ptr<nt::NetworkTable> localTbl =
      nt_inst.GetTable("vision/localization");

  // Make a sanity check topic and an entry to publish/read from it; set initial
  // value
  nt::StringTopic sanitycheck = visionTbl->GetStringTopic("sanitycheck");
  nt::StringEntry sanitycheckEntry =
      sanitycheck.GetEntry("DEFAULT SANITYCHECK SET BY JETSON");
  sanitycheckEntry.Set("Hello, world. I've been set up!");

  // Other vision topics
  nt::BooleanTopic robot_pos_goodTopic =
      localTbl->GetBooleanTopic("robot_pos_good");
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

  // Camera warmup - dropping several first frames to let auto-exposure
  // stabilize
  rs2::frameset frames;

  for (int i = 0; i < 30; i++) {
    frames = pipe.wait_for_frames();
  }

  Mat matframe(Size(640, 480), CV_8UC3,
               (uint8_t *)frames.get_color_frame().get_data(), Mat::AUTO_STEP);
  Mat gray(Size(640, 480), CV_8UC1);

  Matrix3f poseRotationMatrix;
  Vector3f poseAngles;

  // Translation of tag relative to camera
  double linX;
  double linY;
  double linZ;

  double poseErr;

  // Timing
  TickMeter tm;
  TickMeter looptm;

  bool showmode = false;

  double last_rotZ = 0;

  while (true) {
    looptm.reset();
    looptm.start();
    tm.reset();
    tm.start();
    errno = 0;

    // Make sure networktables is working
    sanitycheckEntry.Set("We have entered the loop (on the Jetson).");

    tm.stop();
    tm.reset();
    tm.start();
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

    tm.stop();
    tm.reset();
    tm.start();

    // Do the detecting
    zarray_t *detections = apriltag_detector_detect(td, &im);
    tm.stop();
    tm.reset();
    tm.start();

    if (errno == EAGAIN) {
      printf("Unable to create the %d threads requested.\n", td->nthreads);
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

      // Filter so it doesn't use detections close to the edge
      bool shouldskip = false;
      for (int i = 0; i < 4; i++) {
        if (in_margin(det->p[i])) {
          shouldskip = true;
          break;
        }
      }
      if (shouldskip) {
        robot_pos_goodEntry.Set(false);
        continue;
      }

      // Tag found
      robot_pos_goodEntry.Set(true);

      // Apriltag pose estimation
      /*
      apriltag_pose_t pose;
      apriltag_detection_info_t info;
      info.det = det;
      info.tagsize = TAG_SIZE;
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
      rotZ += M_PI;

      if (rotZ < 0.873 || rotZ > 5.411)
      {
          // Send relevant info to networkTables (it's negative so it's
      relative to tag) robot_xEntry.Set(-linX); robot_yEntry.Set(-linY);
          robot_thetaEntry.Set(rotZ);
      }

      // Print info
      printf("Translation\n X: %f\n Y: %f\n Z: %f\n", linX, linY, linZ);
      printf("Rotation\n Around X: %f\n Around Y: %f\n Around Z: %f\n", rotX,
      rotY, rotZ);

      printf("Pose Error: %f\n\n\n", poseErr);

      // Rotation matrix
      printf("Rotation\n    %f | %f | %f\n    %f | %f | %f\n    %f | %f |
      %f\n\n", pose.R->data[0], pose.R->data[1], pose.R->data[2],
             pose.R->data[3], pose.R->data[4], pose.R->data[5],
             pose.R->data[6], pose.R->data[7], pose.R->data[8]);
             */

      getRelativeTranslationRotation(det, TAG_SIZE, CAM_FX, CAM_FY, CAM_CX,
                                     CAM_CY, tag_trans, tag_rot);
      tm.stop();
      tm.reset();
      tm.start();

      wRo_to_euler(tag_rot, rotX, rotZ, rotY);
      tm.stop();
      tm.reset();
      tm.start();

      // cout << "Translation\n " << tag_trans * 10 * 39.37008 << endl;
      // cout << "Rotation\n " << tag_rot << endl;
      // printf("Better Rotation\n Around X: %f\n Around Y: %f\n Around Z:
      // %f\n\n", rotX, rotY, rotZ);
      linX = tag_trans(1) * 10;
      linY = tag_trans(0) * 10;
      linZ = tag_trans(2) * 10;

      printf("X: %f\nY: %f\nZ:%f\n", linX, linY, linZ);
      printf("Rotation: %f\n", rotZ * 180 / M_PI);

      if (fabs(rotZ) < 0.1) {
        printf("#########################################################\n");
        if (fabs(last_rotZ) > 0.2) {
          continue;
        }
      }

      last_rotZ = rotZ;
      // Send relevant info to networkTables (it's negative so it's relative to
      // tag)
      robot_xEntry.Set(linX);
      robot_yEntry.Set(-linY);
      robot_thetaEntry.Set(-rotZ);
      tm.stop();
      tm.reset();
      tm.start();

      hamm_hist[det->hamming]++;
      total_hamm_hist[det->hamming]++;

      // Draw detection outline
      line(matframe, Point(det->p[0][0], det->p[0][1]),
           Point(det->p[1][0], det->p[1][1]), Scalar(0xff, 0x00, 0x00), 2);
      line(matframe, Point(det->p[0][0], det->p[0][1]),
           Point(det->p[3][0], det->p[3][1]), Scalar(0x00, 0xff, 0x00), 2);
      line(matframe, Point(det->p[1][0], det->p[1][1]),
           Point(det->p[2][0], det->p[2][1]), Scalar(0x00, 0x00, 0xff), 2);
      line(matframe, Point(det->p[2][0], det->p[2][1]),
           Point(det->p[3][0], det->p[3][1]), Scalar(0xff, 0x00, 0xff), 2);

      line(matframe, Point(IMG_MARGIN, DEPTH_HEIGHT - IMG_MARGIN),
           Point(DEPTH_WIDTH - IMG_MARGIN, DEPTH_HEIGHT - IMG_MARGIN),
           Scalar(0xff, 0xff, 0xff), 2);
      line(matframe, Point(IMG_MARGIN, DEPTH_HEIGHT - IMG_MARGIN),
           Point(IMG_MARGIN, IMG_MARGIN), Scalar(0xff, 0xff, 0xff), 2);
      line(matframe, Point(DEPTH_WIDTH - IMG_MARGIN, DEPTH_HEIGHT - IMG_MARGIN),
           Point(DEPTH_WIDTH - IMG_MARGIN, IMG_MARGIN),
           Scalar(0xff, 0xff, 0xff), 2);
      line(matframe, Point(IMG_MARGIN, IMG_MARGIN),
           Point(DEPTH_WIDTH - IMG_MARGIN, IMG_MARGIN),
           Scalar(0xff, 0xff, 0xff), 2);

      // Label the tag
      stringstream ss;
      ss << det->id;
      String text = ss.str();
      int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
      double fontscale = 1.0;
      int baseline;
      Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
      putText(matframe, text,
              Point(det->c[0] - textsize.width / 2,
                    det->c[1] + textsize.height / 2),
              fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

      tm.stop();
      tm.reset();
      tm.start();
    }

    apriltag_detections_destroy(detections);

    imshow("Tag Detections", matframe);
    tm.stop();
    tm.start();

    // if (waitKey(1) == 'q')
    // break;

    looptm.stop();
    cout << "Total Time: " << looptm.getTimeMilli() << endl;
    cout << endl << endl;
  }

  apriltag_detector_destroy(td);
  tag16h5_destroy(tf);

  return 0;
}
