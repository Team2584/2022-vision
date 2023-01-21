#include "main.h"
#include "Cameras.h"

using namespace std;
using namespace cv;

int main(void)
{
    depthCamera depth(DEPTH_RED, 640, 480, 60);

    while (true)
    {
        depth.getFrame();
        depth.findCones();

        char ch = waitKey(1);

        if (ch == 'q')
            break;
    }
}
