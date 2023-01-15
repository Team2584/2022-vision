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

typedef struct camInfo
{
    camPosOffset offset;
    cv::Vec<double, 5> distCoeffs;
    cv::Matx33d camMatx;
} camInfo;

class abstractCamera
{
  protected:
    int width;
    int height;
    int fps;

  public:
    camInfo info;
    virtual cv::Mat getFrame() = 0;
    virtual void setManualExposure(int exposuretime) = 0;
    virtual void setAutoExposure() = 0;
    void setPosOffset(double x = 0, double y = 0, double z = 0, double theta = 0,
                      double elevAngle = 0)

    {
        info.offset.x = x;
        info.offset.y = y;
        info.offset.z = z;
        info.offset.theta = theta;
        info.offset.elevAngle = elevAngle;
    }

    void setDistCoeffs(double c0 = 0, double c1 = 0, double c2 = 0, double c3 = 0, double c4 = 0)
    {
        info.distCoeffs[0] = c0;
        info.distCoeffs[1] = c1;
        info.distCoeffs[2] = c2;
        info.distCoeffs[3] = c3;
        info.distCoeffs[4] = c4;
    }

    void setCamParams(double fx, double fy, double cx, double cy)
    {
        info.camMatx << 0, 0, 0, 0, 0, 0, 0, 0, 1;
        info.camMatx(0, 0) = fx;
        info.camMatx(1, 1) = fy;
        info.camMatx(0, 2) = cx;
        info.camMatx(1, 2) = cy;
    }
};

class flirCamera : public abstractCamera
{
  private:
    // Flir camera globals
    Spinnaker::SystemPtr flirSystem = Spinnaker::System::GetInstance();
    Spinnaker::CameraList flirCamList = flirSystem->GetCameras();
    int height = 540;
    int width = 720;
    Spinnaker::CameraPtr pCam = nullptr;

  public:
    flirCamera(int camNum);
    ~flirCamera();
    void setManualGain(double value);
    void setAutoGain();

    void setManualExposure(int exposuretime);
    void setAutoExposure();
    cv::Mat getFrame();
};

class depthCamera : public abstractCamera
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

class usbCamera : public abstractCamera
{
  private:
    cv::VideoCapture cap;

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
