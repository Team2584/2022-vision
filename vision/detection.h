#ifndef detection_h
#define detection_h

#include "Cameras.h"
#include "graphics_helpers.h"
#include "pose_estimation.h"
#include <apriltag/apriltag.h>
#include <opencv2/opencv.hpp>

int getPoses(cv::Mat grayFrame, cv::Mat colorFrame, camInfo *cam, apriltag_detector_t *td,
             zarray_t *poses);

#endif
