#ifndef WFGROUDTRUTH_H
#define WFGROUDTRUTH_H

#include "WindowTool/WindowFeature.h"

#include "Segmentation/Segmentation.h"
#include "Segmentation/Segment.h"
#include "MatchHandler.h"

#include "GroundTruth/FrameGT.h"

class WFGroudTruth : public WindowFeature
{
private:
    // Some flags:
    bool showThresholdSegmentation;
    bool showMatch;
    bool showHomographyMatchs;
    bool showOpticalFlowMatchs;
    bool showContinuumFeatureMatchs;
    bool showSpatialMatchs;
    bool showHomographyMouse;
    bool showGaussians;
    bool showRegionsOfInterest;
    bool showKMeansResult;

    int selecGauss;

    // Thresholds
    int threshold1,threshold2;

    // Some tools
    ConfigLoader config;
    Segmentation segmentation;

    MatchHandler matchHandler;

private:

    void switchShowGaussians();
    void switchShowMatch();
    void switchShowRegionsOfInterese();

    void removeSelection();

    FrameGT * frame(unsigned id);
public:
    WFGroudTruth();

    void createGaussian(FrameGT *frame,Mat &img16Bits);

    void saveGroundTruth(const char *fileName);
    void loadGroundTruth(const char *fileName);

    void makeCurrentMosaic();

    void acceptHomographyMatchs();
    void acceptOpticalFlowMatchs();
    void acceptContinuumMatchs();

    // WindowFeature interface
public:
    Frame *newFrame(const string &fileName, unsigned frameNumber);
    void keyPress(char c);
    void mouseEvent(int event, int x, int y);
    void renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId);
    void renderFrameTogether(Mat &screen,
                             const Scalar_<float> &el, unsigned leftFrameId,
                             const Scalar_<float> &er, unsigned rightFrameId);
    void processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId);

};

#endif // WFGROUDTRUTH_H
