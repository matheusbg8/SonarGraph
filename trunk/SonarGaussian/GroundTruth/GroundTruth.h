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

#include "GroundTruth/FrameGT.h"
#include "Segmentation/Segmentation.h"
#include "Segmentation/Segment.h"
#include "MatchHandler.h"


// ===== DEBUG SECTION ==========

// #define GROUND_TRUTH_MOUSE_DEBUG
// #define GROUND_TRUTH_KEYBOARD_DEBUG
// #define GROUND_TRUTH_MEMORY_DEBUG
// #define GROUND_TRUTH_TRACKING_DEBUG

// ==== END DEBUG SECTION =======

typedef pair<unsigned, unsigned> PUU;

class GroundTruth
{
public:
    int threshold1,threshold2;
    string windowName;
    string windowControlName;
    string groundTruthFileName;
    Size windowSize;


    Mat result, // Dysplayed Image with left and right frame together
        imgFr1, imgFr2, // Left and Right Frames
        pixelIntensity,
        insonificationCorrection;
    unsigned insonificationMean;

    // Used for k-means auto threshold
    unsigned maxKindOfPixelAnalised, // max distint intensity will be analyzed
             nPixelIntensity;  // distinc pixel intensity analysed by k-maens

    vector<FrameGT> frames; // Frames description
    int currentFrame0, currentFrame1;   // Current frame

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
    bool showIndividualFrameLeft;
    bool showIndividualFrameRight;
    bool showColorMosaic;
    unsigned short useColors;
    int intensity;

    // User Selections
    int selecGauss,selecFrame;
    Point2f mousePosition; // Last mouse position

    // Transformation between imgFrs and display img*/
    // Scalar( xScale , yScale, xTranslation , yTranslation)
    Scalar_<float> el, er; // lef_img and right_img


    ConfigLoader config;
    Segmentation segmentation;

    MatchHandler matchHandler;

    GroundTruth();
    ~GroundTruth();

    void colorName(char *str);

    void saveCurrentImag(const char *fileName);

    void saveGroundTruth(const char *fileName);
    void saveEmptyFrames(const char *fileName);

    void loadGroundTruth(const char *fileName);
    bool loadEmptyFrames(const char *fileName);

    void mainLoop();
    void start();

    void computeAutoThreshold(Mat &img16Bits, int *tr1, int *tr2);
    void computeThreshold(Mat &srcImg, Mat &destImg);

    void drawGaussian(Mat &img, Gaussian &g, const Scalar &lineColor, const Scalar &txtColor,
                      int id, bool drawElipses=true, bool drawVertexID=true, bool drawElipsesAngle=false);

    bool findMatchByIntersectGaussian(unsigned ufr, Gaussian &u, unsigned vfr,
                                      Gaussian **uFound, Gaussian **vFound);

    bool findMatchByIntersectGaussian(unsigned uFrId, Gaussian &u, unsigned vFrId,
                                      int *uIdFound, int *vIdFound);

    bool findMatch(unsigned ufr, float *ux, float *uy, unsigned vfr, float *vx, float *vy, float prec);
    unsigned nMatchs(unsigned ufr, unsigned vfr);
    void buildMatchPoints(unsigned srcFr, unsigned destFr, vector<Point2f> &src, vector<Point2f> &dst);

    void renderImgsOnResult();

    void createFrameRelationGraph(vector<vector<unsigned> > &graph);
    void clearNoMatchFrames();

    bool loadCurrentFrame();
    void loadNextFrame();
    void loadPreviewFrame();

    void selectGaussOnFrame(int x, int y, int *frame, int *gauss);
    void select(int x, int y);
    void removeSelection();

    void acceptHomographyMatchs();
    void acceptOpticalFlowMatchs();
    void acceptContinuumMatchs();

    void makeCurrentMosaic();

    void createGaussian(int frame, int x, int y, Gaussian *g);
    void createGaussian(FrameGT *frame,Mat &img16Bits);
    void createGaussian(Segment *seg , Gaussian *g);

    void mouseUP(int x, int y);
    void mouseEvent(int event, int x, int y);

    void tb1(int);
    void tb2(int);
    void tb3(int);

    void switchShowGaussians();
    void switchShowMatch();
    void switchShowRegionsOfInterese();


    Mat mMeam;
    Mat meanCount;

    void addToMean();
    void showMeam();
    void cleamMean();

    void applyMeanCorrection(Mat &imgGray_16bits);
};

// TrackBars
void on_tbTh1(int v, void* dt);
void on_tbTh2(int v, void* dt);
void on_tbTh3(int v, void* dt);

// Mouse
void onMouse(int event, int x, int y, int, void* dt);

// Buttons
void on_GT_ShowGaussian(int state, void* userData);
void on_GT_ShowMatch(int state, void* userData);
void on_GT_ShowRegionsOfInterest(int state, void* userData);


#endif // GROUNDTRUTH_H
