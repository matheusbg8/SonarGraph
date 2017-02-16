#ifndef CLOSELOOPTESTER_H
#define CLOSELOOPTESTER_H

#include "WindowTool/Frame.h"
#include "Sonar/Sonar.h"
#include "Sonar/SonarDescritor.h"
#include "GraphMatcher/GraphMatcher.h"

#include <vector>
#include <string>

using namespace std;

class CloseLoopTester
{
    vector<Frame> frames;
    vector<SonarDescritor*> sd;
    string datasetPath;
    unsigned jump;

public:
    CloseLoopTester(const string &datasetPath, unsigned jump=1);

    bool loadFrames(const char *fileName);
    void describeFrames();

    void saveDescriptions(const char *fileName);
    void loadDescriptions(const char *fileName);

    void computeMatchs(unsigned windowJump, unsigned start=0, unsigned end=0);

};

#endif // CLOSELOOPTESTER_H
