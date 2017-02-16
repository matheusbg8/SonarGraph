#include "Chart.h"
#include <algorithm>

#include "Drawing/Drawing.h"

Chart::Chart(unsigned width, unsigned height):
    m_width(width),
    m_height(height)
{
    clear();
}

void Chart::setSize(unsigned widht, unsigned height)
{
    m_width= widht;
    m_height = height;
}

void Chart::addPoint(float x, float y)
{
    addPoint(data.size()-1,x,y);
}

void Chart::addPoint(unsigned label, float x, float y)
{
    m_minX = std::min(m_minX,x);
    m_maxX = std::max(m_maxX,x);

    m_minY = std::min(m_minY,y);
    m_maxY = std::max(m_maxY,y);

    data[label].push_back(PFF(x,y));
}

unsigned Chart::newLabel(PlotType plotType, const Scalar &color, int thickness)
{
    data.push_back( vector<pair<float,float> >());
    dataInfo.push_back(ChartInfo());

    ChartInfo &info = dataInfo.back();

    info.plotType = plotType;
    info.color = color;
    info.thickness = thickness;

    return data.size()-1;
}

void Chart::clear()
{
    m_maxX=0.f;
    m_minX=99999.9f;
    m_maxY=0.f;
    m_minY=99999.9f;
    data.clear();
    dataInfo.clear();
}
