#include "WindowFeature.h"
#include "WindowTool/WindowTool.h"

WindowFeature::WindowFeature()
{
}

void WindowFeature::start()
{
}

Frame *WindowFeature::newFrame(const string &fileName, unsigned frameNumber)
{
    return new Frame(fileName, frameNumber);
}

void WindowFeature::keyPress(char c)
{

}

void WindowFeature::mouseEvent(int event, int x, int y)
{

}

void WindowFeature::renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{

}

void WindowFeature::renderFrameTogether(Mat &screen, const Scalar_<float> &el, const Scalar_<float> &er)
{

}

void WindowFeature::processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{

}
