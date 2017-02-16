#ifndef TEXTRENDER_H
#define TEXTRENDER_H

#include "ObjectRender.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>


using namespace std;
using namespace cv;



/**
 * @brief This class grab text render information
 *
 */
class TextRender: public ObjectRender
{
    string _txt;
    Point2f _orig;
    int _fontFace;
    double _fontScale;
    Scalar _color;
    int _thickness,_lineType;

public:
    TextRender(const char *txt="Empty", Point2f orig = Point2f(0, 0),
               int fontFace=FONT_HERSHEY_COMPLEX, double fontScale=0.5,
               Scalar color=Scalar(0.f,255.f,0.f),
               int thickness=1, int lineType=8);


    void setPosition(const Point2f p);
    void setText(const char *txt);
    void setColor(const Scalar &color);

    // ObjectRender interface
public:
    void render(Mat &img);
};

#endif // TEXTRENDER_H
