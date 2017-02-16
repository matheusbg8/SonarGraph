#ifndef SONARTESTWINDOW_H
#define SONARTESTWINDOW_H

#include "WindowTool/WindowFeature.h"

class SonarTestWindow : public WindowFeature
{
public:
    SonarTestWindow();

    // WindowFeature interface
public:
    void keyPress(char c);
    void renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rishId);
    void renderFrameTogether(Mat &screen,
                             const Scalar_<float> &el, unsigned leftFrameId,
                             const Scalar_<float> &er, unsigned rightFrameId);

    void processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId);
};

#endif // SONARTESTWINDOW_H
