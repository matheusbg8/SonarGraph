#ifndef SINGLEIMAGEWINDOW_H
#define SINGLEIMAGEWINDOW_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

class SingleImageFeature;

class SingleImageWindow
{
public:
    Mat screen;
    bool mouseDown,mouseDrag;
    Size windowSize;
    string sourceWindowName;
    Point2f lasMousePosition,
            currentMousePosition;

    vector<SingleImageFeature*> sfts;


    void propagateMouseClick(int x, int y);
    void propagateMouseDrag(int x, int y);
    void propagateRender(Mat &imgBgr);
    void propagateKeyPress(char c);
    void propagateStart();

public:
    SingleImageWindow();

    void start();

    void readKeyboard();

    void refreshScreen();
    void resetScren();

    void addFeature(SingleImageFeature *f);

    void mouseCalback(int event, int x, int y, int flags);
};


// Mouse
void on_SIW_mouse(int event, int x, int y, int flags, void* dt);

#endif // SINGLEIMAGEWINDOW_H
