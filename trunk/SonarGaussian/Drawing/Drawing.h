#ifndef DRAWING_H
#define DRAWING_H

#include <iostream>
#include <vector>
#include <string>

#include <cstdio>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Sonar/Gaussian.h"
#include "Sonar/SonarDescritor.h"

class Chart;

using namespace std;
using namespace cv;


/**
 * @brief This class is used to perform draw operations,
 * it's composed by static methods to do drawing.
 *
 */
class Drawing
{
public:

    static const Scalar color[]; /**< It's a color palette used by some drawing methods */
    static const unsigned nColor; /**< It store the size of color palettes */

    /**
     * @brief Draw two imagens together.
     *
     *   The transformation parameters is a quadruple with
     * two scales and two translations as follow:
     * [ x ratio , y ratio , x translation, y translation ]
     *
     * where:
     *  x ration is result image width / original image width.
     *  y ration is result image height / original image height.
     *  x translation is the displacement of original image to result image.
     *  y translation is the displacement of original image to result image.
     *
     * @param windowSize - Size of final image
     * @param leftBGR8img - Left image
     * @param rightBGR8img - Right image
     * @param result - Resultant image
     * @param el - Tranformation parameters from left image to result image.
     * @param er - Transformation parameters from right image to result image.
     */
    static void drawImgsTogether(const Size &windowSize, Mat &leftBGR8img, Mat &rightBGR8img, Mat &result,
                                 Scalar_<float> *el, Scalar_<float> *er);

    static void convertFrameMatch(vector<vector<pair<unsigned, unsigned> > > &frMatch,
                                  unsigned fr, vector<pair<unsigned, unsigned> > *matchs);

    static void drawMatchings(Mat &BGRImg, const Scalar_<float> &el, const Scalar_<float> &er, vector<Gaussian> &gl,
                              vector<Gaussian> &gr,
                              vector<MatchInfo> &matchs,
                              const Scalar &color);

    static void drawLineMatch(Mat &BGRImg, Scalar_<float> &el, Scalar_<float> &er,
                              const Point2f &begin, const Point2f &end,
                              const Scalar &color, int thickness=1);

    static void drawLineMatch(Mat &BGRImg, Scalar_<float> &el, Scalar_<float> &er,
                              const vector<Point2f> &begin, const vector<Point2f> &end,
                              const Scalar &color, int thickness=1);

    static void drawGaussian(Mat &img, Gaussian &g,
                             const Scalar &lineColor, const Scalar &txtColor,
                             int id,
                             bool drawElipses=true, bool drawVertexID=true,
                             bool drawElipsesAngle=false);

    static void drawGaussianText(Mat &img, Gaussian &g,
                             const Scalar &txtColor,
                             const char * txt,
                             const Point2f &txtPosition);

    static void drawGaussians(Mat &img, vector<Gaussian> &gs,
                             const Scalar &lineColor, const Scalar &txtColor,
                             bool drawElipses=true, bool drawVertexID=true,
                             bool drawElipsesAngle=false);

    static void drawGaussianDots(Mat &img, Gaussian &g,
                                 int id,
                                 const Scalar &dotsColor, const Scalar &txtColor,
                                 bool drawVertexID=true, bool drawElipsesAngle=false,
                                 bool linkDots=false);

    static void drawGaussiansDots(Mat &img, vector<Gaussian> &g,
                                  const Scalar &dotsColor, const Scalar &txtColor,
                                  bool drawVertexID=true, bool drawElipsesAngle=false,
                                  bool linkDots=true);

    static void polarPlot(Mat &result, vector<double> &normData,
                          Size imgSize, const vector<string> &axisLabels=vector<string>());

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

    static void plot(const Chart &chart, Mat &result);

    // ========= Sonar Descriptor Method =================
    static void drawDescriptor(Mat colorImg, SonarDescritor *sd, bool drawEdges = true,
                               bool drawEdgesAngle=false, bool drawEdgesInvAngle=false,
                               bool drawElipses=false, bool drawVertexID=true, bool drawElipsesAngle=false);

    static void drawDescriptorColor(Mat colorImg, SonarDescritor *sd, const Scalar &color, const Scalar &txtColor,
                               bool drawEdges = true, bool drawEdgesAngle=false, bool drawEdgesInvAngle=false,
                               bool drawElipses=false, bool drawVertexID=false, bool drawElipsesAngle=false);

    static void drawVertex(Mat colorImg, SonarDescritor *sd,
                           unsigned vertexID,
                           const Rect &drawingArea,
                           const Scalar &color, const Scalar &txtColor,
                           bool drawEdgeInfo=false);

    static void drawVertex(Mat colorImg, SonarDescritor *sd,
                           unsigned vertexID,
                           const Scalar &color, const Scalar &txtColor,
                           bool drawEdgeInfo=false);

    /**
     * @brief Draw a individual vertex match between one pair of vertices.
     *
     * @param colorImg - BGR8 color image (result).
     * @param drawingArea - Area of drawing.
     * @param sd1 - Description of acoustic image of vertex 1.
     * @param sd2 - Description of acoustic image of vertex 2.
     * @param vertex1Id - Vertex 1 ID.
     * @param vertex2Id - Vertex 2 ID.
     * @param edgeMatchInfo - Informations about the match.
     * @param color - Line color.
     * @param txtColor - Texts Color.
     * @param thickness - Line thickness.
     */
    static void drawVertexMatch(Mat &colorImg, const Size2i &drawingArea,
                                const SonarDescritor &sd1, const SonarDescritor &sd2,
                                unsigned vertex1Id, unsigned vertex2Id,
                                const vector<MatchInfoWeighted> &edgeMatchInfo,
                                bool drawEdgeInfo, bool drawMatchInfo,
                                const Scalar &color, const Scalar &txtColor,
                                int thickness);

    /**
     * @brief Draw each edge matched of each pair of vertices found
     *
     * @param bgrImg1 - BGR8 Acoustic image of first description sd1.
     * @param bgrImg2 - BGR8 Acoustic image of second descriptor sd2.
     * @param sd1 - First descriptor.
     * @param sd2 - Second descriptor.
     * @param matchInfo - Matches informations.
     * @param thickness - thickness of line used to draw edges.
     */
    static void drawGraphVertexMatchs(Mat bgrImg1, Mat bgrImg2,
                                SonarDescritor &sd1, SonarDescritor &sd2,
                                vector<MatchInfoExtended> &matchInfo,
                                int thickness);

    static void drawGraphVertexMatchs(Mat bgrImg1, Mat bgrImg2,
                                SonarDescritor &sd1, SonarDescritor &sd2,
                                MatchInfoExtended &matchInfo,
                                int thickness);

};

#endif // DRAWING_H
