#ifndef WINDOWPAINT_H
#define WINDOWPAINT_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

class PaintFeature;

class WindowPaint
{
protected:
    void propagateMouseDragEvent(int x, int y, Point2f p);
    void propagateMouseDragEndEvent(int x, int y, Point2f p);

    void propagateDrawEndImg(Mat &img8bIts);
    void propagateDrawEndContour(vector<Point2f> &contour);

public:
    Mat gray_8bits;
    unsigned nContourns;
    bool mouseDown;
    Size windowSize;
    string sourceWindowName;
    Point2f lasMousePosition,
            currentMousePosition;
    vector<vector<Point2f> > contours;

    vector<PaintFeature*> pfts;

    void computAndDrawImageMoments(Mat screen, const Point2f &p);
    void computAndDrawContournsMoments(Mat screen, const Point2f &p);

public:
    WindowPaint();

    void start();

    void readKeyboar();
    void resetScren();
    void refreshScreen();

    void mouseCalback(int event, int x, int y, int flags);

    void addPaintFeature(PaintFeature *pf);
};

void windowPaint_mouse_callback(int event, int x, int y, int flags, void *object);


#endif // WINDOWPAINT_H
