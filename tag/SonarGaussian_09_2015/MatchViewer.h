#ifndef MATCHVIEWER_H
#define MATCHVIEWER_H

#include "Sonar.h"
#include "GraphMatcher/GraphMatcher.h"
#include "GroundTruth.h"
#include "Segmentation.h"

// ===== DEBUG SECTION ==========
// #define MATCHVIEWER_TRACKING_DEBUG


// ==== END DEBUG SECTION =======

class MatchViewer
{
private:
    bool loadFileNames(const string & listFileName, vector<string>& fileNames);

public:
    MatchViewer();
    ~MatchViewer();

    void standardExecution();

    void createGroudTruth();

    void compareWithGroundTruth();

    bool computeAllMatches();

    void matchTest();

    void gaussianTest();

    void computeAllMatchs();

};

#endif // MATCHVIEWER_H
