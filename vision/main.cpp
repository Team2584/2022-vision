#include "main.h"
#include "Cameras.h"
#include "detection.h"
#include "graphics_helpers.h"
#include "pose_estimation.h"

using namespace std;
using namespace cv;

int total_hamm_hist[HAMM_HIST_MAX];
int hamm_hist[HAMM_HIST_MAX];

int main()
{
    // flirCamera flir(0);
    // depthCamera depth_blue(DEPTH_BLUE, 640, 480, 60);
    depthCamera depth_red(DEPTH_RED, 640, 480, 60);

    /**********************************************************************************************
     * AprilTags Setup *
     *******************/

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    tf = tag16h5_create();

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family_bits(td, tf, HAMMING_NUMBER);

    td->quad_decimate = QUAD_DECIMATE;
    td->quad_sigma = QUAD_SIGMA;
    td->nthreads = NTHREADS;
    td->debug = APRIL_DEBUG;
    td->refine_edges = REFINE_EDGES;

    memset(total_hamm_hist, 0, sizeof(int) * HAMM_HIST_MAX);

    /**********************************************************************************************
     * Network Tables Setup *
     ************************/

    // Create networktables instan8ce and a table for vision
    nt::NetworkTableInstance nt_inst = nt::NetworkTableInstance::GetDefault();

    // Setup networktable client
    nt_inst.StartClient4("jetson client");
    nt_inst.SetServerTeam(2584);
    nt_inst.StartDSClient();
    nt_inst.SetServer("host", NT_DEFAULT_PORT4);

    // Create tables
    shared_ptr<nt::NetworkTable> visionTbl = nt_inst.GetTable("vision");
    shared_ptr<nt::NetworkTable> localTbl = nt_inst.GetTable("vision/localization");
    shared_ptr<nt::NetworkTable> objTbl = nt_inst.GetTable("vision/objects");

    // Make a sanity check topic and an entry to publish/read from it; set initial
    // value
    nt::IntegerTopic sanitycheck = localTbl->GetIntegerTopic("sanitycheck");
    nt::IntegerEntry sanitycheckEntry = sanitycheck.GetEntry(0, {.periodic = 0.01});
    sanitycheckEntry.Set(1);

    // Other vision topics
    nt::DoubleArrayTopic robot_pose_Topic = localTbl->GetDoubleArrayTopic("poseArray");
    nt::DoubleArrayEntry robot_pose_Entry = robot_pose_Topic.GetEntry({});

    nt::DoubleArrayTopic cone_pos_Topic = objTbl->GetDoubleArrayTopic("conePos");
    nt::DoubleArrayPublisher cone_pos_Entry = cone_pos_Topic.GetEntry({});

    nt::DoubleArrayTopic pole_pos_Topic = objTbl->GetDoubleArrayTopic("polePos");
    nt::DoubleArrayPublisher pole_pos_Entry = pole_pos_Topic.GetEntry({});

    /**********************************************************************************************
     * THE LOOP *
     ************/
    int counter = 2;
    double poseNum = 0;
    double coneNum = 0;

    while (true)
    {
        errno = 0;
        memset(hamm_hist, 0, sizeof(hamm_hist));

        // Make sure networktables is working
        sanitycheckEntry.Set(counter);
        counter++;

        // flir.getFrame();
        // depth_blue.getFrame();
        depth_red.getFrame();
        chrono::time_point frameTime = chrono::steady_clock::now();

        if (errno == EAGAIN)
        {
            printf("Unable to create the %d threads requested.\n", td->nthreads);
            continue;
        }

        std::vector<robot_position> poses =
            getPoses(depth_red.grayFrame, depth_red.colorFrame, &depth_red.info, td);

        for (unsigned int i = 0; i < poses.size(); i++)
        {
            robot_position pos = poses[i];
            cout << "X: " << pos.x << endl;
            cout << "Y: " << pos.y << endl;
            cout << "Z: " << pos.z << endl << endl;
            cout << "Theta: " << pos.theta << endl;

            double ms = time_since(frameTime);
            vector<double> poseVector = {pos.x, pos.y, pos.z, pos.theta, ms, poseNum};
            robot_pose_Entry.Set(poseVector);
            nt_inst.Flush();
            poseNum++;
        }

        // Print & send cone info
        pair<double, double> conePos = depth_red.findCones();
        cout << "Cone X: " << conePos.first << endl;
        cout << "Cone Y: " << conePos.second << endl << endl;
        double ms = time_since(frameTime);
        vector<double> coneVector = {conePos.first, conePos.second, ms, coneNum};
        cone_pos_Entry.Set(coneVector);

        // Print & send pole info
        /*
        pair<double, double> polePos = depth_red.findPoles();
        cout << "Pole X: " << polePos.first << endl;
        cout << "Pole Y: " << polePos.second << endl;
        ms = time_since(frameTime);
        vector<double> poleVector = {polePos.first, polePos.second, ms, id}
        pole_pos_Entry.Set(poleVector);
        */

        nt_inst.Flush();

        // Graphics stuff
        // drawMargins(depth_red.colorFrame);
        // imshow("flir", flir.colorFrame);
        // imshow("depth_blue", depth_blue.colorFrame);
        // imshow("depth_red", depth_red.colorFrame);
        // if (waitKey(1) == 'q')
        // break;
    }

    apriltag_detector_destroy(td);
    tag16h5_destroy(tf);
    destroyAllWindows();

    return 0;
}

// Return time elapsed since passed time_point in microseconds
double time_since(std::chrono::time_point<std::chrono::steady_clock> start)
{
    chrono::time_point end = chrono::steady_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
    return (double)duration.count();
}
