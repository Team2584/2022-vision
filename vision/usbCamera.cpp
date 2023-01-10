#include "Cameras.h"

using namespace std;
using namespace cv;

usbCamera::usbCamera(int camNum, int width, int height, int fps) : cap{camNum}
{
    if (cap.isOpened())
    {
        cap.set(CAP_PROP_FRAME_WIDTH, width);
        cap.set(CAP_PROP_FRAME_HEIGHT, height);
        cap.set(CAP_PROP_FPS, fps);
    }
}

usbCamera::~usbCamera()
{
}

void usbCamera::setManualExposure(int exposuretime)
{
    // TODO impl
}

void usbCamera::setAutoExposure()
{
    // TODO impl
}

void usbCamera::setManualFocus()
{
    // TODO impl
}

void usbCamera::setAutoFocus()
{
    // TODO impl
}

cv::Mat usbCamera::getFrame()
{
    cv::Mat frame;
    cap >> frame;
    return frame;
}
