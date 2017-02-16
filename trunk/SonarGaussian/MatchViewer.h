#ifndef MATCHVIEWER_H
#define MATCHVIEWER_H

#include "Sonar/Sonar.h"
#include "GraphMatcher/GraphMatcher.h"
#include "GroundTruth.h"
#include "Segmentation.h"

#include "CloseLoopAnaliseResult.h"

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

    void compareWithGorundTruth2();

    void testWindowTool();    

    void generateMatlabMatrix(const char *groundTruthFileName);

    void closeLoopDetection(const string &datasetPath, unsigned start=0u, unsigned end=0u);

    void closeLoopAnaliseResult();

    void testGaussianDescriptionFeature();

    void testPaintWindow();

    void testSVM();

    void gtLoopTester();


    void gtTopologicalMatch();

};

#endif // MATCHVIEWER_H
