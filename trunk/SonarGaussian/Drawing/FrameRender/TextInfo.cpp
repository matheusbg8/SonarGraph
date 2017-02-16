#include "TextRender.h"

TextRender::TextRender(const char *txt, Point2f orig,
                   int fontFace, double fontScale,
                   Scalar color,
                   int thickness, int lineType):
    _txt(txt),
    _orig(orig),
    _fontFace(fontFace),
    _fontScale(fontScale),
    _color(color),
    _thickness(thickness),
    _lineType(lineType)
{

}

void TextRender::setPosition(const Point2f p)
{
    _orig = p;
}

void TextRender::setText(const char *txt)
{
    _txt = txt;
}

void TextRender::setColor(const Scalar &color)
{
    _color = color;
}

void TextRender::render(Mat &img)
{
    putText(img,_txt,
           _orig,
           _fontFace,_fontScale,
           _color,
           _thickness,_lineType,false);
}
