#ifndef ROBOMAPGENERATOR_H
#define ROBOMAPGENERATOR_H

#include "BagEditor/ConfigLoader/ConfigLoader.h"

#include<string>
#include<map>
#include<cstdio>

#include <rosbag/bag.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

class RoboMapGenerator
{
public:

    RoboMapGenerator();
    virtual ~RoboMapGenerator();

};

#endif // ROBOMAPGENERATOR_H
