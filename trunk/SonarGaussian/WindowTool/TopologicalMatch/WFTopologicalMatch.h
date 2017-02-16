#ifndef WFTOPOLOGICALMATCH_H
#define WFTOPOLOGICALMATCH_H

#include "WindowTool/WindowFeature.h"
#include "WindowTool/FrameSD.h"
#include "Sonar/Sonar.h"
#include "GraphMatcher/GraphMatcher.h"

class WFTopologicalMatch : public WindowFeature
{
protected:
    FrameSD *frame(unsigned i);

    vector<SonarDescritor*> sds;
    ConfigLoader sonarConfig;
    Sonar sonar;
    GraphMatcher gm;
    vector<MatchInfo> matchInfo;

    int leftSelecGaussian, rightSelecGaussian;

    bool showVertexMatch;

public:
    WFTopologicalMatch();

    // WindowFeature interface
public:
    void start();
    Frame *newFrame(const string &fileName, unsigned frameNumber);
    void keyPress(char c);
    void mouseEvent(int event, int x, int y);
    void renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId);
    void renderFrameTogether(Mat &screen,
                             const Scalar_<float> &el, unsigned leftFrameId,
                             const Scalar_<float> &er, unsigned rightFrameId);
    void processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId);
};

#endif // WFTOPOLOGICALMATCH_H
