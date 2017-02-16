#include "SingleImageWindow.h"
#include "WindowTool/SingleImageWindow/SingleImageFeature.h"


void SingleImageWindow::propagateMouseClick(int x, int y)
{
    for(unsigned i = 0 ; i < sfts.size(); i++)
    {
        sfts[i]->mouseClick(x,y);
    }
}

void SingleImageWindow::propagateMouseDrag(int x, int y)
{
    for(unsigned i = 0 ; i < sfts.size(); i++)
    {
        sfts[i]->mouseDrag(x,y);
    }
}

void SingleImageWindow::propagateRender(Mat &imgBgr)
{
    for(unsigned i = 0 ; i < sfts.size(); i++)
    {
        sfts[i]->render(imgBgr);
    }
}

void SingleImageWindow::propagateKeyPress(char c)
{
    for(unsigned i = 0 ; i < sfts.size(); i++)
    {
        sfts[i]->keyPress(c);
    }
}

void SingleImageWindow::propagateStart()
{
    for(unsigned i = 0 ; i < sfts.size(); i++)
    {
        sfts[i]->start();
    }
}

SingleImageWindow::SingleImageWindow():
    mouseDown(false),
    mouseDrag(false),
    windowSize(600,600)
{

}

void SingleImageWindow::start()
{
    /// Create Window
    namedWindow( sourceWindowName.c_str(), CV_WINDOW_AUTOSIZE );

    /// Create mouse callback
    setMouseCallback(sourceWindowName,on_SIW_mouse, this);

    /// Initialize image with black background
    resetScren();
    refreshScreen();

    propagateStart();

    /// Start keyborad
    readKeyboard();
}

void SingleImageWindow::readKeyboard()
{
    char c = 0;
    c = waitKey();
    while(c != 27)
    {
        propagateKeyPress(c);
        c = waitKey();
    }

}

void SingleImageWindow::refreshScreen()
{
    propagateRender(screen);

    /// Show in a window
    imshow(sourceWindowName, screen);
}

void SingleImageWindow::resetScren()
{
    screen = Mat(windowSize.height,windowSize.width,CV_8UC3, Scalar(0,0,0));
}

void SingleImageWindow::addFeature(SingleImageFeature *f)
{
    sfts.push_back(f);
    f->_SIW = this;
}

void SingleImageWindow::mouseCalback(int event, int x, int y, int flags)
{
    switch(event)
    {
    case CV_EVENT_LBUTTONDOWN:
        mouseDown = true;
    break;
    case CV_EVENT_LBUTTONUP:
        mouseDown = false;
        if(mouseDrag == true)
        {
            mouseDrag = false;
        }else
        {
            propagateMouseClick(x,y);
            refreshScreen();
        }
    break;
    case CV_EVENT_MOUSEMOVE:
        if(mouseDown == true)
        {
            mouseDrag = true;

            propagateMouseDrag(x,y);
            refreshScreen();
        }
    break;
    }
}

void on_SIW_mouse(int event, int x, int y, int flags, void *dt)
{
    SingleImageWindow *siw = (SingleImageWindow*) dt;
    siw->mouseCalback(event,x,y,flags);
}
