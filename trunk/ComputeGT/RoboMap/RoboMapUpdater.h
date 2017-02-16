#ifndef ROBOMAPUPDATER_H
#define ROBOMAPUPDATER_H

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

class RoboMapUpdater
{
private:

public:

    RoboMapUpdater();
    virtual ~RoboMapUpdater();

};

#endif // ROBOMAPUPDATER_H
