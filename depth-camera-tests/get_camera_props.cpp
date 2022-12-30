
// include the librealsense C++ header file
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/h/rs_types.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <librealsense2/rs.hpp>

// include OpenCV header file
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main()
{
    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_ANY, 60);

    //Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile selection = pipe.start(cfg);

    // Get depth units
    rs2::depth_sensor sensor = selection.get_device().first<rs2::depth_sensor>();
    double scale = sensor.get_depth_scale();

    // Get intrinsic properties of depth stream
    rs2::video_stream_profile depth_stream = selection.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    std::pair<int, int> depth_res = std::make_pair(depth_stream.width(), depth_stream.height());
    rs2_intrinsics depth_i = depth_stream.get_intrinsics();
    // Focal center: (depth_i.ppx, depth_i.ppy)
    // Focal length depth_i.fx, depth_i.fy

    // Get intrinsic properties of color stream
    rs2::video_stream_profile color_stream = selection.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    std::pair<int, int> color_res= std::make_pair(color_stream.width(), color_stream.height());
    rs2_intrinsics color_i = color_stream.get_intrinsics();
    // Focal center: (color_i.ppx, color_i.ppy) (floats)
    // Focal length color_i.fx, color_i.fy (floats)

    printf("Depth Units: %f\n", scale);
    printf("Color Stream Props:\n");
    printf("    resolution: %ix%i\n", color_res.first, color_res.second);
    printf("    cx: %f\n    cy: %f\n    fx: %f\n    fy: %f\n", color_i.ppx, color_i.ppy, color_i.fx, color_i.fy);
    printf("Depth Stream Props:\n");
    printf("    resolution: %ix%i\n", depth_res.first, depth_res.second);
    printf("    cx: %f\n    cy: %f\n    fx: %f\n    fy: %f\n", depth_i.ppx, depth_i.ppy, depth_i.fx, depth_i.fy);
}
