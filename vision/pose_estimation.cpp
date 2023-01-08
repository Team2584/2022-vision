#include "pose_estimation.h"
#include "main.h"

using namespace std;

//----------- From AprilNav -----------------------------
// Normalize angle to be within the interval [-pi,pi].
inline double standardRad(double t)
{
    if (t >= 0.)
    {
        t = fmod(t + M_PI, M_TWOPI) - M_PI;
    }
    else
    {
        t = fmod(t - M_PI, -M_TWOPI) + M_PI;
    }
    return t;
}

// Convert rotation matrix to Euler angles
void wRo_to_euler(const Eigen::Matrix3d &wRo, double &yaw, double &pitch, double &roll)
{
    yaw = standardRad(atan2(wRo(1, 0), wRo(0, 0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2, 0), wRo(0, 0) * c + wRo(1, 0) * s));
    roll = standardRad(atan2(wRo(0, 2) * s - wRo(1, 2) * c, -wRo(0, 1) * s + wRo(1, 1) * c));
}

Eigen::Matrix4d getRelativeTransform(apriltag_detection_t *det, double tag_size, double fx,
                                     double fy, double px, double py)
{
    std::vector<cv::Point3f> objPts;
    std::vector<cv::Point2f> imgPts;
    double s = tag_size / 2.;
    objPts.push_back(cv::Point3f(-s, -s, 0));
    objPts.push_back(cv::Point3f(s, -s, 0));
    objPts.push_back(cv::Point3f(s, s, 0));
    objPts.push_back(cv::Point3f(-s, s, 0));

    std::pair<float, float> p1(det->p[0][0], det->p[0][1]);
    std::pair<float, float> p2(det->p[1][0], det->p[1][1]);
    std::pair<float, float> p3(det->p[2][0], det->p[2][1]);
    std::pair<float, float> p4(det->p[3][0], det->p[3][1]);
    imgPts.push_back(cv::Point2f(p1.first, p1.second));
    imgPts.push_back(cv::Point2f(p2.first, p2.second));
    imgPts.push_back(cv::Point2f(p3.first, p3.second));
    imgPts.push_back(cv::Point2f(p4.first, p4.second));

    cv::Mat rvec, tvec;
    cv::Matx33d cameraMatrix(fx, 0, px, 0, fy, py, 0, 0, 1);
    cv::Vec4f distParam(0, 0, 0, 0); // all 0?
    cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
    cv::Matx33d r;
    cv::Rodrigues(rvec, r);
    Eigen::Matrix3d wRo;
    wRo << r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2);

    Eigen::Matrix4d T;
    T.topLeftCorner(3, 3) = wRo;
    T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
    T.row(3) << 0, 0, 0, 1;

    return T;
}

void getRelativeTranslationRotation(apriltag_detection_t *det, double tag_size, double fx,
                                    double fy, double px, double py, Eigen::Vector3d &trans,
                                    Eigen::Matrix3d &rot)
{
    Eigen::Matrix4d T = getRelativeTransform(det, tag_size, fx, fy, px, py);

    // converting from camera frame (z forward, x right, y down) to
    // object frame (x forward, y left, z up)
    Eigen::Matrix4d M;
    M << 0, 0, 1, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 1;
    Eigen::Matrix4d MT = M * T;
    // translation vector from camera to the April tag
    trans = MT.col(3).head(3);
    // orientation of April tag with respect to camera: the camera
    // convention makes more sense here, because yaw,pitch,roll then
    // naturally agree with the orientation of the object
    rot = T.block(0, 0, 3, 3);
}

// ------------------ NOT from AprilNav ---------------------------------
Eigen::Vector2d getRealTranslationRotation(double theta, double x, double y)
{
    printf("X: %f\nY: %f\nTheta: %f\n", x, y, theta * 180 / M_PI);
    Eigen::Matrix2d R;
    R(0, 0) = -cos(theta);
    R(1, 0) = -sin(theta);
    R(0, 1) = sin(theta);
    R(1, 1) = -cos(theta);
    Eigen::Vector2d T;
    T << x, y;
    cout << "Rotation Matrix:\n" << R << endl;
    cout << "Point:\n" << T << endl;
    Eigen::Vector2d trans = R * T;
    cout << "Rotated Point:\n" << trans << endl;
    return trans;
}

void getRobotPosition(apriltag_detection_t *det, robot_position *pos)
{
    Eigen::Vector3d tag_trans;
    Eigen::Matrix3d tag_rot;
    double rotX;
    double rotY;
    double rotZ;

    getRelativeTranslationRotation(det, TAG_SIZE, CAM_FX, CAM_FY, CAM_CX, CAM_CY, tag_trans,
                                   tag_rot);

    wRo_to_euler(tag_rot, rotX, rotZ, rotY);

    // cout << "Translation\n " << tag_trans * 39.37008 << endl;
    // cout << "Rotation\n " << tag_rot << endl;
    // printf("Better Rotation\n Around X: %f\n Around Y: %f\n Around Z: %f\n\n", rotX,
    // rotY, rotZ);

    double linX = tag_trans(1);
    double linY = tag_trans(0) /* + 0.381*/;
    // double linZ = tag_trans(2);

    Eigen::Vector2d point = getRealTranslationRotation(-rotZ, linX, linY);

    pos->x = -point(0);
    pos->y = point(1);
    pos->theta = rotZ;
}
