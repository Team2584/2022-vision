#include "detection.h"

void detectTags(cv::Mat frame, apriltag_detector_t *td, zarray_t *prev_detections)
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
        zarray_add_all(prev_detections, detections);
}
