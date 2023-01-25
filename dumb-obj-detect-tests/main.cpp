#include "main.h"
#include "Cameras.h"

using namespace std;
using namespace cv;

int main(void)
{
    double counter = 2;
    // Create networktables instan8ce and a table for vision
    nt::NetworkTableInstance nt_inst = nt::NetworkTableInstance::GetDefault();

    // Setup networktable client
    nt_inst.StartClient4("jetson client");
    nt_inst.SetServerTeam(2584);
    nt_inst.StartDSClient();
    nt_inst.SetServer("host", NT_DEFAULT_PORT4);

    // Create tables
    shared_ptr<nt::NetworkTable> visionTbl = nt_inst.GetTable("vision");
    shared_ptr<nt::NetworkTable> localTbl = nt_inst.GetTable("vision/objects");

    // Make a sanity check topic and an entry to publish/read from it; set initial
    // value
    nt::DoubleTopic sanitycheck = localTbl->GetDoubleTopic("sanitycheck");
    nt::DoubleEntry sanitycheckEntry = sanitycheck.GetEntry(counter);
    counter++;
    sanitycheckEntry.Set(counter);

    // Other vision topics
    nt::DoubleTopic cone_x_Topic = localTbl->GetDoubleTopic("coneX");
    nt::DoubleTopic cone_y_Topic = localTbl->GetDoubleTopic("coneY");
    nt::DoubleEntry cone_x_Entry = cone_x_Topic.GetEntry(-200);
    nt::DoubleEntry cone_y_Entry = cone_y_Topic.GetEntry(-200);

    depthCamera depth(DEPTH_RED, 640, 480, 60);

    while (true)
    {
        sanitycheckEntry.Set(counter);
        counter++;

        depth.getFrame();
        double pole = depth.findPoles();
        // depth.findCones();
        // imshow("thing", depth.colorFrame);

        // imshow("Image", depth.colorFrame);
        nt_inst.Flush();

        // if (cone_coords.first != 0 || cone_coords.second != 0)
        //{
        // cone_x_Entry.Set(cone_coords.first);
        // cone_y_Entry.Set(cone_coords.second);
        //}

        if (waitKey(25) == 'q')
            break;
    }
}
