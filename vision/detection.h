#ifndef detection_h
#define detection_h

#include <apriltag/apriltag.h>
#include <opencv2/opencv.hpp>

void detectTags(cv::Mat frame, apriltag_detector_t *td, zarray_t *prev_detections);

#endif
