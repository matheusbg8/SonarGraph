#ifndef GTLOOPFRAME_H
#define GTLOOPFRAME_H

#include <opencv2/core/core.hpp>

using namespace cv;

class GTLoopFrame
{
public:
    Point2f p, c;
    float heading;

    GTLoopFrame(double timeStamp,
                const Point2f &p, const Point2f &c, float heading);
};

#endif // GTLOOPFRAME_H
