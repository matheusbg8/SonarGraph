#ifndef GAUSSIANRENDER_H
#define GAUSSIANRENDER_H

#include "Sonar/Gaussian.h"
#include "Drawing/FrameRender/ObjectRender.h"
#include "Drawing/FrameRender/TextRender.h"

class GaussianRender : public ObjectRender
{
    Point2f _position;
    Size2f _axes;
    double _ang;
    Scalar _color;
    int _thickness, _lineType;


    TextRender _idLabel;

public:
    GaussianRender();
    GaussianRender(Gaussian &g, unsigned id);

    void setGaussian(Gaussian &g);
    void setId(unsigned id);
    void setTxtColor(const Scalar &color);
    void setColor(const Scalar &color);

    // RenderObject interface
public:
    void render(Mat &img);
};

#endif // GAUSSIANRENDER_H
