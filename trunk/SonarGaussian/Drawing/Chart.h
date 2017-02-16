#ifndef CHART_H
#define CHART_H

#include <vector>
using namespace std;

#include "Drawing/Drawing.h"


class Chart
{
public:
    enum PlotType
    {
        PLOT_LINE,
        PLOT_CONTINUOS_LINE,
        PLOT_CIRCLE
    };

    typedef struct
    {
        PlotType plotType;
        Scalar color;
        int thickness;
    }ChartInfo;

private:

    typedef pair<float,float> PFF;

    float m_ex, m_ey,
          m_maxX,m_minX,
          m_maxY,m_minY;

    unsigned m_width, m_height;

    vector< vector<pair<float,float> > > data;
    vector<ChartInfo> dataInfo;

public:
    Chart(unsigned width=800, unsigned height=600);

    void setSize(unsigned widht, unsigned height);

    void addPoint(float x, float y);
    void addPoint(unsigned label,float x, float y);
    unsigned newLabel(PlotType plotType, const Scalar &color, int thickness=1);

    void clear();

    friend void Drawing::plot(const Chart &chart, Mat &result);
};

#endif // CHART_H
