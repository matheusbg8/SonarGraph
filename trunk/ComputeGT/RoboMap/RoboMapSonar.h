#ifndef ROBOMAPSONAR_H
#define ROBOMAPSONAR_H

#include "RoboMap/RoboMapElement.h"

#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

class RoboMapSonar : public RoboMapElement
{
public:
    Point2f p,c;
    float heading, bearing, range;

    RoboMapSonar(const Point2f &p= Point(0.f,0.f),
                 float heading=0.f, float bearing=130.f, float range=30.f);

    // RoboMapElement interface
public:
    void render(Mat &img);
};

#endif // ROBOMAPSONAR_H
