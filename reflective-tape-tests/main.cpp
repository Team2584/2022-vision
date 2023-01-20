#include "main.h"
#include "Cameras.h"

using namespace std;
using namespace cv;

bool checkBlock(Mat frame, int starty, int startx)
{
    for (int y = starty; y <= starty + 5; y++)
    {
        for (int x = startx; x <= starty + 5; x++)
        {
            if (frame.at<Vec3b>(y, x)[2] < 100)
            {
                return false;
            }
        }
    }
    return true;
}

int main()
{
    depthCamera depth(DEPTH_RED, 640, 480, 60);
    double alpha = 0.5;
    int beta = 0;

    cout << "* Enter the alpha value [1.0-3.0]: ";
    cin >> alpha;
    cout << "* Enter the beta value [0-100]: ";
    cin >> beta;

    while (true)
    {
        depth.getFrame();

        Mat orig = depth.colorFrame;

        Mat newFrame = Mat::zeros(orig.size(), orig.type());

        // cvtColor(orig, newFrame, COLOR_BGR2YCrCb);

        for (int y = 0; y < orig.rows; y++)
        {
            for (int x = 0; x < orig.cols; x++)
            {
                for (int c = 0; c < orig.channels(); c++)
                {
                    newFrame.at<Vec3b>(y, x)[c] =
                        saturate_cast<uchar>(alpha * orig.at<Vec3b>(y, x)[c] + beta);
                }
            }
        }

        vector<Point> points;

        Mat hsvFrame;
        cvtColor(newFrame, hsvFrame, COLOR_BGR2HSV);

        for (int y = 0; y < hsvFrame.rows; y += 5)
        {
            for (int x = 0; x < hsvFrame.cols; x += 5)
            {
                if (checkBlock(hsvFrame, y, x))
                {
                    points.push_back(Point(y, x));
                }
            }
        }

        for (int i = 0; i < points.size() - 1; i++)
        {
            Line(newFrame, points[i], points[i + 1], Scalar(255, 255, 255));
        }

        imshow("depth", newFrame);

        if (waitKey(1) == 'q')
            break;
    }

    return 0;
}
