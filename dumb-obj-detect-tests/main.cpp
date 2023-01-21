#include "main.h"
#include "Cameras.h"

using namespace std;
using namespace cv;

int main(void)
{
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
    nt::IntegerTopic sanitycheck = localTbl->GetIntegerTopic("sanitycheck");
    nt::IntegerEntry sanitycheckEntry = sanitycheck.GetEntry(0, {.periodic = 0.01});
    sanitycheckEntry.Set(1);

    // Other vision topics
    nt::DoubleTopic cone_x_Topic = localTbl->GetDoubleTopic("coneX");
    nt::DoubleEntry cone_x_Entry = cone_x_Topic.GetEntry(-2, {.periodic = 0.01});

    depthCamera depth(DEPTH_RED, 640, 480, 60);

    int counter = 2;

    while (true)
    {
        sanitycheckEntry.Set(counter);
        counter++;

        depth.getFrame();
        double cone_x = depth.findCones();
        if (cone_x != -1)
        {
            cone_x_Entry.Set(cone_x);
        }

        if (waitKey(1) == 'q')
            break;
    }
}
