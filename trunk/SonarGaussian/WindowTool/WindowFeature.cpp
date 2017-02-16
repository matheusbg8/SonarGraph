#include "WindowFeature.h"
#include "WindowTool/WindowTool.h"

WindowFeature::WindowFeature()
{
}

WindowFeature::~WindowFeature()
{

}

void WindowFeature::start()
{
}

Frame *WindowFeature::newFrame(const string &fileName, unsigned frameNumber)
{
    return new Frame(fileName, frameNumber);
}

void WindowFeature::selectedFrame(int frameId)
{

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

void WindowFeature::renderFrameTogether(Mat &screen,
                                        const Scalar_<float> &el, unsigned leftFrameId,
                                        const Scalar_<float> &er, unsigned rightFrameId)
{

}

void WindowFeature::processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{

}
