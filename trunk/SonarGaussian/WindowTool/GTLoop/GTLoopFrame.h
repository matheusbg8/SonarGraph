#ifndef GTLOOPFRAME_H
#define GTLOOPFRAME_H

#include "WindowTool/Frame.h"
#include <opencv2/core/core.hpp>

using namespace cv;

class GTLoopFrame : public Frame
{
public:
    Point2f p, c;
    float heading;

    GTLoopFrame(const string& fileName,unsigned frameNumber,
                double timeStamp,
                const Point2f &p, const Point2f &c, float heading);
};

#endif // GTLOOPFRAME_H
