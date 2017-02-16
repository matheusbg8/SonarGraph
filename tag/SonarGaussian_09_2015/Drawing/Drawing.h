#ifndef DRAWING_H
#define DRAWING_H

#include <iostream>
#include <vector>
#include <string>

#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Gaussian.h"

using namespace std;
using namespace cv;

class Drawing
{
public:

    static const Scalar color[6];
    static const unsigned nColor;

    static void drawImgsTogether(const Size &windowSize, Mat &leftBGR8img, Mat &rightBGR8img, Mat &result,
                                 Scalar_<float> *el, Scalar_<float> *er);

    static void convertFrameMatch(vector<vector<pair<unsigned, unsigned> > > &frMatch,
                                  unsigned fr, vector<pair<unsigned, unsigned> > *matchs);

    static void drawMatchings(Mat &BGRImg, const Scalar_<float> &el, const Scalar_<float> &er,
                              vector<Gaussian> &gr, vector<Gaussian> &gl,
                              vector<pair<unsigned, unsigned> > &matchs,
                              const Scalar &color);

    static void drawLineMatch(Mat &BGRImg, Scalar_<float> &el, Scalar_<float> &er,
                              const Point2f &begin, const Point2f &end,
                              const Scalar &color, int thickness=1);

    static void drawLineMatch(Mat &BGRImg, Scalar_<float> &el, Scalar_<float> &er,
                              const vector<Point2f> &begin, const vector<Point2f> &end,
                              const Scalar &color, int thickness=1);

    static void drawGaussian(Mat &img, Gaussian &g, const Scalar &lineColor, const Scalar &txtColor,
                             int id, bool drawElipses=true, bool drawVertexID=true, bool drawElipsesAngle=false);

    static void plot(Mat &result, const Size &size,
                     vector<float> &x,vector<float> &y,
                     const Scalar &color, Scalar *plotInfo = 0x0,
                     int thickness=1);

    static void makePlotInfo(Mat &result, const Size &size,
                     float xMin, float xMax, float yMin, float yMax,
                     Scalar &plotInfo);

    static void holdPlot(const Scalar &plotInf, Mat &result,
                         vector<float> &x,vector<float> &y,
                         const Scalar &color, int thickness=1);

    static void holdPlotCircle(const Scalar &plotInf, Mat &result,
                         vector<float> &x,vector<float> &y,
                         const Scalar &color, int thickness=1);

};

#endif // DRAWING_H
