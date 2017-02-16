#ifndef CLOSELOOPTESTER_H
#define CLOSELOOPTESTER_H

#include "WindowTool/Frame.h"
#include "Sonar/Sonar.h"
#include "Sonar/SonarDescritor.h"
#include "GraphMatcher/GraphMatcher.h"

#include <vector>
using namespace std;

class CloseLoopTester
{

    vector<Frame> frames;
    vector<SonarDescritor*> sd;

public:
    CloseLoopTester();

    bool loadFrames(const char *fileName);

    void describeFrames();

    void computeMatchs(unsigned windowJump, unsigned start=0, unsigned end=0);


};

#endif // CLOSELOOPTESTER_H
