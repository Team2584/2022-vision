#include "main.h"
#include "Cameras.h"
#include "graphics_helpers.h"
#include "pose_estimation.h"

using namespace std;
using namespace cv;
using namespace Eigen;

bool shouldIgnoreDetection(apriltag_detection_t *det)
{
    // Only use valid tag detections
    if (det->id > 8 || det->id < 1)
        return true;

    // Filter so it doesn't use detections close to the edge
    for (int i = 0; i < 4; i++)
    {
        if (in_margin(det->p[i]))
        {
            return true;
        }
    }

    return false;
}

int main(void)
{
    // depthCamera depth(0, 640, 480, 60);
    flirCamera flir(0);

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

    int total_hamm_hist[HAMM_HIST_MAX];
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

    // Make a sanity check topic and an entry to publish/read from it; set initial
    // value
    nt::IntegerTopic sanitycheck = visionTbl->GetIntegerTopic("sanitycheck");
    nt::IntegerEntry sanitycheckEntry = sanitycheck.GetEntry(0, {.periodic = 0.01});
    sanitycheckEntry.Set(1);

    // Other vision topics
    nt::BooleanTopic robot_pos_goodTopic = localTbl->GetBooleanTopic("robot_pos_good");
    nt::DoubleTopic robot_xTopic = localTbl->GetDoubleTopic("x");
    nt::DoubleTopic robot_yTopic = localTbl->GetDoubleTopic("y");
    nt::DoubleTopic robot_thetaTopic = localTbl->GetDoubleTopic("theta");

    nt::BooleanEntry robot_pos_goodEntry = robot_pos_goodTopic.GetEntry(false, {.periodic = 0.01});
    nt::DoubleEntry robot_xEntry = robot_xTopic.GetEntry(0.0, {.periodic = 0.01});
    nt::DoubleEntry robot_yEntry = robot_yTopic.GetEntry(0.0, {.periodic = 0.01});
    nt::DoubleEntry robot_thetaEntry = robot_thetaTopic.GetEntry(0.0, {.periodic = 0.01});

    /**********************************************************************************************
     * THE LOOP *
     ************/

    Mat frame(720, 540, CV_8UC3);
    Mat gray(720, 540, CV_8UC1);

    Matrix3f poseRotationMatrix;
    Vector3f poseAngles;

    double poseErr;

    int counter = 2;

    while (true)
    {
        errno = 0;
        int hamm_hist[HAMM_HIST_MAX];
        memset(hamm_hist, 0, sizeof(hamm_hist));

        // Make sure networktables is working
        sanitycheckEntry.Set(counter);
        counter++;

        // Grab a frame
        printf("point 1-\n");
        gray = flir.getFrame();
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        printf("point 2-\n");
        cout << gray.cols << "|" << gray.rows << endl;
        printf("point 3-\n");

        // Make an image_u8_t header from the frame
        image_u8_t im = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data,
        };
        printf("point 4-\n");

        // Detect Tags
        zarray_t *detections = apriltag_detector_detect(td, &im);
        printf("point 6-\n");
        if (errno == EAGAIN)
        {
            printf("Unable to create the %d threads requested.\n", td->nthreads);
            exit(-1);
        }

        printf("point 7-\n");
        robot_pos_goodEntry.Set(false);

        printf("point 8-\n");
        // Loop through detections
        for (int i = 0; i < zarray_size(detections); i++)
        {
            // Get the detection
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            if (shouldIgnoreDetection(det))
                continue;

            robot_position pos;
            getRobotPosition(det, &pos);

            // Tag found
            robot_pos_goodEntry.Set(true);

            // Send relevant info to networkTables (it's negative so it's relative to
            // tag)
            robot_xEntry.Set(pos.x);
            robot_yEntry.Set(pos.y);
            robot_thetaEntry.Set(pos.theta);

            hamm_hist[det->hamming]++;
            total_hamm_hist[det->hamming]++;

            labelDetections(frame, det);
        }

        printf("point 9-\n");
        drawMargins(gray);
        // imshow("Tag Detections", gray);

        apriltag_detections_destroy(detections);

        printf("point 10-\n");
        // if (waitKey(1) == 'q')
        // break;

        // nt_inst.Flush();
    }

    apriltag_detector_destroy(td);
    tag16h5_destroy(tf);

    return 0;
}
