#ifndef WINDOWFEATURETEST_H
#define WINDOWFEATURETEST_H

#include "WindowTool/WindowFeature.h"


/**
 * @brief This class was created just by test
 * WindowsFeature and WIndowsTools class
 *
 */
class WindowFeatureTest : public WindowFeature
{
public:
    WindowFeatureTest();

    // WindowFeature interface
public:
    void keyPress(char c);
    void mouseEvent(int event, int x, int y);
    void renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId);
    void renderFrameTogether(Mat &screen,
                             const Scalar_<float> &el, unsigned leftFrameId,
                             const Scalar_<float> &er, unsigned rightFrameId);
    void processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId);
};

#endif // WINDOWFEATURETEST_H
