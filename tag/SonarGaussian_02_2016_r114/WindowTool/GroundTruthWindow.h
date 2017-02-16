#ifndef GROUNDTRUTHWINDOW_H
#define GROUNDTRUTHWINDOW_H

#include "WindowTool/WindowFeature.h"

class GroundTruthWindow : public WindowFeature
{
public:
    GroundTruthWindow();



    // WindowFeature interface
public:
    void keyPress(char c);

    void renderProcess(Mat leftImg, int leftId, Mat rightImg, int rishId);

    void renderFrameTogether(Mat screen, const Scalar_<float> &el, const Scalar_<float> &er);

    void processImages(Mat leftImg, int leftId,
                       Mat rightImg, int rightId);
};

#endif // GROUNDTRUTHWINDOW_H
