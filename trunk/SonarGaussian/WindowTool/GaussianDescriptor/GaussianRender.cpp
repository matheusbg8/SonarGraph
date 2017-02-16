#include "GaussianRender.h"
#include "Drawing/Drawing.h"

GaussianRender::GaussianRender():
    _position(0.f,0.f),
    _axes(0,0),
    _ang(0.0),
    _color(0,255,0),
    _thickness(1),
    _lineType(8)
{

}

GaussianRender::GaussianRender(Gaussian &g, unsigned id):
    _position(0.f,0.f),
    _axes(0,0),
    _ang(0.0),
    _color(0,255,0),
    _thickness(1),
    _lineType(8)
{
    setGaussian(g);
    setId(id);
}

void GaussianRender::setGaussian(Gaussian &g)
{
    _ang = g.ang;
    _position = Point2f(g.x,g.y);
    _axes.width = g.dx;
    _axes.height = g.dy;

    _idLabel.setPosition(Point2f(_position.x+20.f,_position.y+20.f));
}

void GaussianRender::setId(unsigned id)
{
    char str[200];
    sprintf(str,"ID %u", id);
    _idLabel.setText(str);
}

void GaussianRender::setTxtColor(const Scalar &color)
{
    _idLabel.setColor(color);
}

void GaussianRender::setColor(const Scalar &color)
{
    _color = color;
}

void GaussianRender::render(Mat &img)
{
    ellipse(img,
            _position,
            _axes,
            _ang,
            0.0,360.0, // Full elipse
            _color,
            _thickness, _lineType,0);

    _idLabel.render(img);
}
