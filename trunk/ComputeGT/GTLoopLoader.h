#ifndef GTLOOPLOADER_H
#define GTLOOPLOADER_H

#include <string>
#include <vector>
using namespace std;

#include "GTLoopFrame.h"

typedef struct
{
    double timeStamp;
    float x,y,z,rot;
}CSVTFPosition;

typedef struct
{
    unsigned id;
    double timeStamp;
}CSVFrame;

typedef struct
{
    double timeStamp;
    float heading;
}CSVHeading;

class GTLoopLoader
{
public:
    float sonarRange;

    GTLoopLoader();

    vector<CSVTFPosition> csvPosition;
    vector<CSVFrame> csvFrames;
    vector<CSVHeading> csvHeading;

    bool loadPositionFromCSV(const string &positionCSV);
    bool loadFramesFromCSV(const string &frameInfo);
    bool loadHeadingFromCSV(const string &headingCSV);

    void syncFramesData(const string &prefix, vector<GTLoopFrame> &frames);

};

#endif // GTLOOPLOADER_H
