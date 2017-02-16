#ifndef ROBOMAPGFILETLOOPGENERATOR_H
#define ROBOMAPGFILETLOOPGENERATOR_H

#include<string>
#include<map>
#include<cstdio>

#include <rosbag/bag.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <opencv2/core/core.hpp>

#include <Tools/RoboMap/RoboMapGTLoopGenerator.h>

using namespace cv;
using namespace std;

class RoboMapFileGTLoopGenerator: public RoboMapGTLoopGenerator
{    
    FILE *fMatch, *fSimilar;
    string similarFileName, matchFileName;

public:

    RoboMapFileGTLoopGenerator(const string &similarFileName = "GTSimilarMatchs.csv",
                               const string &matchFileName = "GTMatchs.csv");

    ~RoboMapFileGTLoopGenerator();

    // RoboMapGTLoopGenerator interface
public:
    void generateGTLoop(unsigned srcId, double time,
                        const vector<pair<unsigned, float> > &similaRegion,
                        const vector<pair<unsigned, float> > &match);
};

#endif // ROBOMAPGFILETLOOPGENERATOR_H
