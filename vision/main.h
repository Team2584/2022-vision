#ifndef main_h
#define main_h

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
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/RawTopic.h>
#include <networktables/StringTopic.h>

extern "C"
{
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
}

// Constants
#define M_TWOPI 6.283185307

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

#define IMG_MARGIN 20

#endif
