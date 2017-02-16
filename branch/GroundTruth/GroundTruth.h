#ifndef GROUNDTRUTH_H
#define GROUNDTRUTH_H

#include <iostream>
#include <vector>
#include <string>

#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

#include "Frame.h"
#include "Segmentation/Segmentation.h"

class GroundTruth
{
public:
    int threshold1,threshold2;
    string windowName;
    Size windowSize;

    Mat img, result, imgFr1, imgFr2, pixelIntensity;
    unsigned maxKindOfPixelAnalised, nPixelIntensity;

    vector<Frame> frames; // Frames description
    int currentFrame0, currentFrame1;   // Current frame

    bool showThresholdSegmentation;
    bool showMatch;
    bool showGaussians;
    bool showKMeansResult;

    int selecFrame, selecGauss;

    float e1x, e1y, d1x, d1y, e2x, e2y, d2x, d2y;

    Segmentation segmentation;

    GroundTruth();
// 0-38   e  47

    void saveGroundTruth(const char *fileName);
    void loadGroundTruth(const char *fileName);
    void loadEmptyFrames(const char *fileName);

    void mainLoop();
    void start();

    void computeAutoThreshold(Mat &img16Bits, int *tr1, int *tr2);
    void computeThreshold(Mat &srcImg, Mat &destImg);

    void drawGaussian(Mat &img, Gaussian &g, const Scalar &lineColor, const Scalar &txtColor,
                      int id, bool drawElipses=true, bool drawVertexID=true, bool drawElipsesAngle=false);

    void renderImgsOnResult();

    void loadCurrentFrame();
    void loadNextFrame();
    void loadPreviewFrame();

    void selectGaussOnFrame(int x, int y, int *frame, int *gauss);
    void select(int x, int y);
    void removeSelection();

    void createGaussian(int frame, int x, int y, Gaussian *g);
    void createGaussian(Frame *frame,Mat &img16Bits);
    void createGaussian(Segment *seg , Gaussian *g);

    void mouseUP(int x, int y);
    void mouseEvent(int event, int x, int y);
    void tb1(int);
    void tb2(int);
};

void on_tbTh1(int v, void* dt);
void on_tbTh2(int v, void* dt);
void onMouse(int event, int x, int y, int, void* dt);


#endif // GROUNDTRUTH_H
