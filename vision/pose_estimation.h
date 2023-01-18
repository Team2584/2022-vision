#ifndef pose_estimation_h
#define pose_estimation_h

#include <Eigen/Eigen>
#include <cmath>

extern "C"
{
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tag16h5.h>
}

#include "Cameras.h"

#define M_TWOPI 2 * M_PI

#define TAG_SIZE 0.1524
#define INCH 39.37

void getRobotPosition(apriltag_detection_t *det, robot_position *pos, camInfo *cam);

#endif
