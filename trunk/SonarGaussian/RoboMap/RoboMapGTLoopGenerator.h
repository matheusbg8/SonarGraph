#ifndef ROBOMAPGTLOOPGENERATOR_H
#define ROBOMAPGTLOOPGENERATOR_H

#include<string>
#include<map>
#include<cstdio>

#include <rosbag/bag.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <opencv2/core/core.hpp>

#include <Tools/RoboMap/RoboMapGenerator.h>

using namespace cv;
using namespace std;

class RoboMapGTLoopGenerator: public RoboMapGenerator
{
public:

    RoboMapGTLoopGenerator();
    virtual ~RoboMapGTLoopGenerator();

    virtual void generateGTLoop(unsigned srcId, double time,
                        const vector<pair<unsigned, float> > &similaRegion,
                        const vector<pair<unsigned, float> > &match) = 0;

};

#endif // ROBOMAPGTLOOPGENERATOR_H
