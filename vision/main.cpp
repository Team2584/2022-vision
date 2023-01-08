#include "main.h"
#include "pose_estimation.h"

using namespace std;
using namespace cv;
using namespace Eigen;

bool in_margin(double p[])
{
    if (p[0] < IMG_MARGIN || p[0] > DEPTH_WIDTH - IMG_MARGIN)
        return true;
    if (p[1] < IMG_MARGIN || p[1] > DEPTH_HEIGHT - IMG_MARGIN)
        return true;
    return false;
}

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
    /**********************************************************************************************
     * DEPTH CAMPERA SETUP *
     ***********************/

    // Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    // Create a configuration for configuring the pipeline with a non default
    // profile
    rs2::config cfg;

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_BGR8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, DEPTH_WIDTH, DEPTH_HEIGHT, RS2_FORMAT_ANY, 60);

    // Instruct pipeline to start streaming with the requested configuration
    pipe.start(cfg);

    /**********************************************************************************************
     * LOGITECH CAMERA SETUP *
     ************************/

    VideoCapture cap(0);
    if (cap.isOpened())
    {
        cap.set(CAP_PROP_FRAME_WIDTH, 640);
        cap.set(CAP_PROP_FRAME_HEIGHT, 480);
        cap.set(CAP_PROP_FPS, 30);
    }

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

    rs2::frameset frames;

    frames = pipe.wait_for_frames();

    Mat matframe(Size(640, 480), CV_8UC3, (uint8_t *)frames.get_color_frame().get_data(),
                 Mat::AUTO_STEP);
    Mat gray(Size(640, 480), CV_8UC1);
    Mat lastgray(Size(640, 480), CV_8UC1);
    Mat diff;

    Matrix3f poseRotationMatrix;
    Vector3f poseAngles;

    double poseErr;

    // Timing
    TickMeter tm;
    TickMeter looptm;

    bool showmode = false;

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
        frames = pipe.wait_for_frames();
        rs2::video_frame frame = frames.get_color_frame();

        // The Mat business is only needed for displaying results
        matframe.data = (uint8_t *)frame.get_data();
        cvtColor(matframe, gray, COLOR_BGR2GRAY);

        // Make an image_u8_t header from the frame
        image_u8_t im = {
            .width = gray.cols,
            .height = gray.rows,
            .stride = gray.cols,
            .buf = gray.data,
        };

        // Detect Tags
        zarray_t *detections = apriltag_detector_detect(td, &im);
        if (errno == EAGAIN)
        {
            printf("Unable to create the %d threads requested.\n", td->nthreads);
            exit(-1);
        }

        robot_pos_goodEntry.Set(false);

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

            tm.stop();
            tm.reset();
            tm.start();

            hamm_hist[det->hamming]++;
            total_hamm_hist[det->hamming]++;

            // Draw detection outline
            line(matframe, Point(det->p[0][0], det->p[0][1]), Point(det->p[1][0], det->p[1][1]),
                 Scalar(0xff, 0x00, 0x00), 2);
            line(matframe, Point(det->p[0][0], det->p[0][1]), Point(det->p[3][0], det->p[3][1]),
                 Scalar(0x00, 0xff, 0x00), 2);
            line(matframe, Point(det->p[1][0], det->p[1][1]), Point(det->p[2][0], det->p[2][1]),
                 Scalar(0x00, 0x00, 0xff), 2);
            line(matframe, Point(det->p[2][0], det->p[2][1]), Point(det->p[3][0], det->p[3][1]),
                 Scalar(0xff, 0x00, 0xff), 2);

            line(matframe, Point(IMG_MARGIN, DEPTH_HEIGHT - IMG_MARGIN),
                 Point(DEPTH_WIDTH - IMG_MARGIN, DEPTH_HEIGHT - IMG_MARGIN),
                 Scalar(0xff, 0xff, 0xff), 2);
            line(matframe, Point(IMG_MARGIN, DEPTH_HEIGHT - IMG_MARGIN),
                 Point(IMG_MARGIN, IMG_MARGIN), Scalar(0xff, 0xff, 0xff), 2);
            line(matframe, Point(DEPTH_WIDTH - IMG_MARGIN, DEPTH_HEIGHT - IMG_MARGIN),
                 Point(DEPTH_WIDTH - IMG_MARGIN, IMG_MARGIN), Scalar(0xff, 0xff, 0xff), 2);
            line(matframe, Point(IMG_MARGIN, IMG_MARGIN),
                 Point(DEPTH_WIDTH - IMG_MARGIN, IMG_MARGIN), Scalar(0xff, 0xff, 0xff), 2);

            // Label the tag
            stringstream ss;
            ss << det->id;
            String text = ss.str();
            int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
            double fontscale = 1.0;
            int baseline;
            Size textsize = getTextSize(text, fontface, fontscale, 2, &baseline);
            putText(matframe, text,
                    Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2),
                    fontface, fontscale, Scalar(0xff, 0x99, 0), 2);

            tm.stop();
            tm.reset();
            tm.start();
        }

        apriltag_detections_destroy(detections);

        imshow("Tag Detections", matframe);
        tm.stop();
        tm.start();

        // if (waitKey(1) == 'q')
        //   break;

        looptm.stop();
        // cout << "Total Time: " << looptm.getTimeMilli() << endl;
        cout << endl << endl;
        nt_inst.Flush();
    }

    apriltag_detector_destroy(td);
    tag16h5_destroy(tf);

    return 0;
}
