#ifndef WINDOWTOOL_H
#define WINDOWTOOL_H

#include "Frame.h"
#include "Segmentation/Segmentation.h"
#include "Segmentation/Segment.h"
#include "MatchHandler.h"
#include "WindowFeature.h"


/**
 * @brief This class provide a window graphic interface for
 * many tasks. It's use a WindowFeature interface.
 *
 */
class WindowTool
{
public:
    int threshold1,threshold2;
    string windowName;
    string windowControlName;
    Size windowSize;    

    Mat result,  /** A Matrix used to display left and right frame together on screen */
        leftImg, righImg,  /** Left and right frames */
        pixelIntensity,
        isonificationPattern,
        bgrMask, grayMask;

    unsigned insonificationMean;

    // Used for k-means auto threshold
    unsigned maxKindOfPixelAnalised, // max distint intensity will be analyzed
             nPixelIntensity;  // distinc pixel intensity analysed by k-maens

    vector<Frame*> frames; // Frames description
    int currentLeftFrame, currentRightFrame;   // Current frame

    bool showIndividualLeftFrame;
    bool showIndividualRightFrame;
    unsigned short useColors;
    int intensity;

    // User Selections
    int selecFrame;
    Point2f mousePosition; // Last mouse position

    // Transformation between imgFrs and display img*/
    // Scalar( xScale , yScale, xTranslation , yTranslation)
    Scalar_<float> el, er; // lef_img and right_img

    vector<WindowFeature*> windowFeatures;

    WindowTool(string windowName="WindowTool", WindowFeature * feature = 0x0);
    ~WindowTool();

    // ====== Window tools events propagation methods =====
    void propagateSelectedFrame();

    // ==================================================
    void addFeature(WindowFeature *wf);
    void clearFetures();

    void clearFrames();

    void colorName(char *str);

    void loadImgMaskAndIsonificationPatern();

    bool loadEmptyFrames(const char *fileName);
    bool saveEmptyFrames(const char *fileName);

    void createIsonificationPatternAndMask(unsigned threadPixel);

    void applyMeanCorrection(Mat &imgGray_16bits);

    bool save(const char *fileName);
    bool load(const char *fileName);

    void mainLoop();
    void start();

    void renderImgsOnResult();

    bool loadCurrentFrame();
    void loadNextFrame();
    void loadPreviewFrame();

    void selectFrame(int x, int y);
    void removeSelection();

    void mouseUP(int x, int y);
    void mouseEvent(int event, int x, int y);

    bool isFrameSelected();
    bool isCurrentRightFrame(unsigned frameId);
    bool isCurrentLeftFrame(unsigned frameId);

    int selectedFrameId();
    Mat &selectedFrameImg();
    int getLeftFrameId();
    int getRighFrameId();

};

// TrackBars (currently don't used)
void on_WT_tbTh1(int v, void* userData);
void on_WT_tbTh2(int v, void* userData);
void on_WT_tbTh3(int v, void* userData);

// Mouse
void on_WT_mouse(int event, int x, int y, int flags, void* dt);

#endif // WINDOWTOOL_H
