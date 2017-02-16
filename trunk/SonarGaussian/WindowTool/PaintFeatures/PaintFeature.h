#ifndef PAINTFEATURE_H
#define PAINTFEATURE_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;


class PaintFeature
{
public:
    PaintFeature();

    virtual void mouseDrag(int x, int y);
    virtual void mouseDragEnd(int x, int y);

    virtual void drawEndImg(Mat &img8Bits);
    virtual void drawEndContour(vector<Point2f> &contour);

};

#endif // PAINTFEATURE_H
