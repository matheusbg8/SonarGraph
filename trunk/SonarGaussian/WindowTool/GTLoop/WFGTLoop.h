#ifndef WFGTLOOP_H
#define WFGTLOOP_H

#include "CSVReader/CSVReader.h"
#include "WindowTool/WindowFeature.h"
#include "WindowTool/GTLoop/GTLoopFrame.h"
#include "RoboMap/RoboMap.h"

#include "WindowTool/GTLoop/GTLoopLoader.h"

class GTLoopEdge
{
public:
    unsigned srcId, dstId;
    float score;
    GTLoopEdge(unsigned srcId=0u, unsigned dstId=0u, float score=0.f):
        srcId(srcId),dstId(dstId),score(score){}
};

class WFGTLoop : public WindowFeature
{
public:
    WFGTLoop();
    ~WFGTLoop();

    vector<GTLoopFrame> m_frames;
    vector<GTLoopEdge> m_matchEdges;
    vector< vector<GTLoopEdge*> >
                    m_match;

    GTLoopLoader csvData;

    RoboMap roboMap;
    string datasetPath;

    float maxDistBetweenCenters,
          maxDistBetweenPositions;

    bool firstPause;

    void showGTLoop();
    bool loadGTLoopWithTimeStamp(const string &file);
    bool loadGTLoop(const string &file);
    void indexMatchs();
    bool createGTLoop();

    bool saveGTLoop(const string &fileName);
    bool saveFrameInfo(const string &fileName);

    void loadWindowTool(const string &imgPrefix);

    void plotFullMapPath();
    void plotRestrictedMapPath(unsigned minNumFeaturesThreshold);


    // WindowFeature interface
public:
    void start();
    void selectedFrame(int frameId);
    void keyPress(char c);
    void mouseEvent(int event, int x, int y);
    void renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId);
    void renderFrameTogether(Mat &screen, const Scalar_<float> &el, unsigned leftFrameId, const Scalar_<float> &er, unsigned rightFrameId);
    void processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId);



};

#endif // WFGTLOOP_H
