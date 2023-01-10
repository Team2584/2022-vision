#ifndef Camera_h
#define Camera_h

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

typedef struct camPosOffset
{
    double x;
    double y;
    double z;
    double theta;
    double elevAngle;
} camPosOffset;

class abstractCamera
{
  protected:
    int width;
    int height;
    int fps;

  public:
    camPosOffset offset;

    virtual cv::Mat getFrame() = 0;
    virtual void setManualExposure(int exposuretime) = 0;
    virtual void setAutoExposure() = 0;
    void setCamPosOffset(double x = 0, double y = 0, double z = 0, double theta = 0,
                         double elevAngle = 0)
    {
        offset.x = x;
        offset.y = y;
        offset.z = z;
        offset.theta = theta;
        offset.elevAngle = elevAngle;
    }
};

class flirCamera : abstractCamera
{
  private:
    int height = 540;
    int width = 720;
    cv::Mat lastframe;
    Spinnaker::ImagePtr frame = nullptr;
    Spinnaker::CameraPtr pCam = nullptr;
    // Flir camera globals
    Spinnaker::SystemPtr flirSystem = Spinnaker::System::GetInstance();
    Spinnaker::CameraList flirCamList = flirSystem->GetCameras();

  public:
    flirCamera(int camNum);
    ~flirCamera();
    void setManualGain(double value);
    void setAutoGain();

    void setManualExposure(int exposuretime);
    void setAutoExposure();
    cv::Mat getFrame();
};

class depthCamera : abstractCamera
{
  private:
    // Create a pipeline which abstracts the camera
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default
    // profile
    rs2::config cfg;

  public:
    depthCamera(int camNum, int width, int height, int fps);
    ~depthCamera();

    void setManualExposure(int exposuretime);
    void setAutoExposure();
    cv::Mat getFrame();
};

class usbCamera : abstractCamera
{
  private:

  public:
    usbCamera(int camNum, int width, int height, int fps);
    ~usbCamera();

    void setManualExposure(int exposuretime);
    void setAutoExposure();
    void setManualFocus();
    void setAutoFocus();
    cv::Mat getFrame();
};

#endif
