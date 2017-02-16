#ifndef WFTOPOLOGICALVERTEXMATCH_H
#define WFTOPOLOGICALVERTEXMATCH_H

#include "WindowTool/WindowFeature.h"
#include "WindowTool/FrameSD.h"
#include "Sonar/Sonar.h"
#include "GraphMatcher/GraphMatcher.h"

class WFTopologicalVertexMatch : public WindowFeature
{
protected:
    FrameSD *frame(unsigned i);

    vector<SonarDescritor*> sds;
    ConfigLoader sonarConfig;
    Sonar sonar;
    GraphMatcher gm;
    vector<MatchInfo> matchInfo;
    MatchInfoExtended VertexMatchInfo;

    int leftSelecGaussian, rightSelecGaussian;

    int selectGaussianOnFrame(int x,  int y , int frameID, float precision=50.f);
    void removeGaussianSelections();

    bool showVertexMatch;

    void explore(unsigned edgeId);
    void selectBestPairOfLeftGaussian();

public:
    WFTopologicalVertexMatch();

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

#endif // WFTOPOLOGICALVERTEXMATCH_H
