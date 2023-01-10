#include "Cameras.h"

using namespace std;

flirCamera::flirCamera(int camNum)
{
    using namespace Spinnaker;
    using namespace Spinnaker::GenApi;
    using namespace Spinnaker::GenICam;

    printf("Setting up flir camera. Ignoring arguments width, height, and fps.\n");

    printf("point 1\n");
    const unsigned int numCameras = flirCamList.GetSize();

    // Finish if there are no cameras
    if (numCameras == 0)
    {
        flirCamList.Clear();
        flirSystem->ReleaseInstance();
        cout << "No flir cameras found" << endl;
        // TODO throw
    }

    if (numCameras < camNum)
    {
        cout << "Unable to access specified camera" << endl;
        // TODO throw
    }

    printf("point 2\n");
    // Select camera
    pCam = flirCamList.GetByIndex(0);
    pCam->Init();

    printf("point 3\n");
    // Retrieve GenICam nodemap
    INodeMap &nodeMap = pCam->GetNodeMap();

    printf("point 4\n");
    // Recieve enumeration node from nodemap
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
        cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl
             << endl;
        // TODO throw
    }

    // Retrieve entry node from enumeration node
    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
        cout << "Unable to set acquisition mode to continuous (entry retrieval). Aborting..."
             << endl
             << endl;
        // TODO throw
    }

    printf("point 5\n");
    // Retrieve integer value from entry node
    const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

    printf("point 6\n");
    // Set integer value from entry node as new value of enumeration node
    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    printf("point 7\n");
    // Retrieve Stream Parameters device nodemap
    Spinnaker::GenApi::INodeMap &sNodeMap = pCam->GetTLStreamNodeMap();

    printf("point 8\n");
    // Retrieve Buffer Handling Mode Information
    CEnumerationPtr ptrHandlingMode = sNodeMap.GetNode("StreamBufferHandlingMode");
    if (!IsAvailable(ptrHandlingMode) || !IsWritable(ptrHandlingMode))
    {
        cout << "Unable to set Buffer Handling mode (node retrieval). Aborting..." << endl << endl;
        // TODO throw
    }

    CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetCurrentEntry();
    if (!IsAvailable(ptrHandlingModeEntry) || !IsReadable(ptrHandlingModeEntry))
    {
        cout << "Unable to set Buffer Handling mode (Entry retrieval). Aborting..." << endl << endl;
        // TODO throw
    }

    printf("point 9\n");
    // Set buffer handling mode
    ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("NewestOnly");
    ptrHandlingMode->SetIntValue(ptrHandlingModeEntry->GetValue());

    printf("point 10\n");
    // Exposure
    pCam->ExposureAuto.SetValue(ExposureAuto_Continuous);

    printf("point 11\n");
    // Gain
    pCam->GainAuto.SetValue(GainAutoEnums::GainAuto_Once);

    printf("point 12\n");
    // Gamma
    pCam->Gamma.SetValue(1.0);

    printf("point 13\n");

    printf("point 14\n");
    pCam->BeginAcquisition();

    lastframe.data = (uint8_t *)pCam->GetNextImage()->GetData();
}

flirCamera::~flirCamera()
{
    pCam = nullptr;
    flirCamList.Clear();
}

void flirCamera::setAutoExposure()
{
    pCam->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Continuous);
}

void flirCamera::setManualExposure(int exposuretime = 0)
{
    // If no argument passed, use auto_once mode (this is the default)
    if (exposuretime == 0)
    {
        pCam->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Once);
    }
    else
    {
        pCam->ExposureAuto.SetValue(Spinnaker::ExposureAuto_Off);
        pCam->ExposureMode.SetValue(Spinnaker::ExposureMode_Timed);
        pCam->ExposureTime.SetValue(exposuretime);
    }
}

void flirCamera::setManualGain(double value = 0)
{
    // If no argument passed, use auto_once mode (this is the default)
    if (value == 0)
    {
        pCam->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Once);
    }
    else
    {
        pCam->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Off);
        pCam->Gain.SetValue(value);
    }
}

void flirCamera::setAutoGain()
{
    pCam->GainAuto.SetValue(Spinnaker::GainAutoEnums::GainAuto_Continuous);
}

cv::Mat flirCamera::getFrame()
{
        printf("point 1---\n");
        printf("point 2---\n");
        Spinnaker::ImageStatus framestatus;
        printf("point 3---\n");

        frame = pCam->GetNextImage();
        printf("point 4---\n");

        framestatus = frame->GetImageStatus();
        printf("point 5---\n");
        if (framestatus != 0)
            printf("FRAME ERROR");

        printf("point 6---\n");
        lastframe.data = (uint8_t *)frame->GetData();
        printf("point 7---\n");
        return lastframe;
}
