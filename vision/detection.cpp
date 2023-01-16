#include "detection.h"
#include "graphics_helpers.h"

bool shouldIgnoreDetection(apriltag_detection_t *det, int frame_width, int frame_height)
{
    // Only use valid tag detections
    if (det->id > 8 || det->id < 1)
        return true;

    // Filter so it doesn't use detections close to the edge
    for (int i = 0; i < 4; i++)
    {
        if (in_margin(det->p[i], frame_width, frame_height))
        {
            return true;
        }
    }
    return false;
}

void detectTags(cv::Mat frame, cv::Mat colorFrame, apriltag_detector_t *td,
                zarray_t *prev_detections)
{
    // Make an image_u8_t header from the frame
    image_u8_t im = {
        .width = frame.cols,
        .height = frame.rows,
        .stride = frame.cols,
        .buf = frame.data,
    };

    // Detect Tags
    zarray_t *detections = apriltag_detector_detect(td, &im);

    // Filter detections and add good ones to array
    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        if (!shouldIgnoreDetection(det, frame.cols, frame.rows))
        {
            zarray_add(prev_detections, &det);
            labelDetections(colorFrame, det);
        }
    }
}
