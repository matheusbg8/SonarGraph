#include "WindowFeatureTest.h"
#include "WindowTool/WindowTool.h"

#include <iostream>
using namespace std;


WindowFeatureTest::WindowFeatureTest()
{
}

void WindowFeatureTest::keyPress(char c)
{
    cout << "Pressed key (" << c << ")" << endl;
}

void WindowFeatureTest::mouseEvent(int event, int x, int y)
{
    cout << "Mouse ";
    switch (event)
    {
    case CV_EVENT_MOUSEMOVE:
        cout << "move ";
        break;
    case CV_EVENT_LBUTTONDOWN:
        cout << "left button down ";
        break;
    case CV_EVENT_RBUTTONDOWN:
        cout << "right button down ";
        break;
    case CV_EVENT_MBUTTONDOWN:
        cout << "middle button down ";
        break;
    case CV_EVENT_LBUTTONUP:
        cout << "left button up ";
        break;
    case CV_EVENT_RBUTTONUP:
        cout << "right button up ";
        break;
    case CV_EVENT_MBUTTONUP:
        cout << "middle button up ";
        break;
    case CV_EVENT_LBUTTONDBLCLK:
        cout << "left button double click ";
        break;
    case CV_EVENT_RBUTTONDBLCLK:
        cout << "right button double click ";
        break;
    case CV_EVENT_MBUTTONDBLCLK:
        cout << "middle button double click ";
        break;
    default:
        cout << "Uknow event! ";
        break;
    }
    cout << " on position "
         << " x=" << x
         << " y=" << y
         << endl;
}

void WindowFeatureTest::renderProcess(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    cout << "Rendering images process ... " << endl;

    cout << "Right Gray image has size: "
         << wt->righImg.cols
         << " , "
         << wt->righImg.rows
         << endl;
}

void WindowFeatureTest::renderFrameTogether(Mat &screen,
                                            const Scalar_<float> &el, unsigned leftFrameId,
                                            const Scalar_<float> &er, unsigned rightFrameId)
{
    cout << "Rendering images together ... " << endl;
}

void WindowFeatureTest::processImages(Mat &leftImg, int leftId, Mat &rightImg, int rightId)
{
    cout << "Pre processing image ... " << endl;
}
