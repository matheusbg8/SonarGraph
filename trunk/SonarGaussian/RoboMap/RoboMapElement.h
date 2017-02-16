#ifndef ROBOMAPELEMENT_H
#define ROBOMAPELEMENT_H

class RoboMap;

#include <opencv2/core/core.hpp>
using namespace cv;

class RoboMapElement
{
protected:
    RoboMap *roboMap;
public:

    RoboMapElement();
    virtual ~RoboMapElement();

    virtual void render(Mat &img) = 0;
};

#endif // ROBOMAPELEMENT_H
