#ifndef SONARDESCRITOR_H
#define SONARDESCRITOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<vector>
#include"Gaussian.h"
#include"GraphLink.h"
#include "SpatialMatchInfo.h"

using namespace std;
using namespace cv;

class SonarDescritor
{
public:
    vector<Gaussian> gaussians;
    vector<vector<GraphLink*> > graph; // pair< dest , ang link north referenced >

    float x, y, ang;

    SonarDescritor();
    ~SonarDescritor();

    static void drawDescriptor(Mat colorImg, SonarDescritor *sd, bool drawEdges = true,
                               bool drawEdgesAngle=false, bool drawEdgesInvAngle=false,
                               bool drawElipses=false, bool drawVertexID=true, bool drawElipsesAngle=false);

    static void drawDescriptorColor(Mat colorImg, SonarDescritor *sd, const Scalar &color, const Scalar &txtColor,
                               bool drawEdges = true, bool drawEdgesAngle=false, bool drawEdgesInvAngle=false,
                               bool drawElipses=false, bool drawVertexID=true, bool drawElipsesAngle=false);

    static void drawVertex(Mat colorImg, SonarDescritor *sd, unsigned vertexID, const Rect &drawingArea, const Scalar &color, const Scalar &txtColor);

    static void drawVertexMatch(Mat bgrImg1, Mat bgrImg2,
                                SonarDescritor &sd1, SonarDescritor &sd2, vector<SpatialMatchInfo> &matchInfo,
                                const Scalar &color, const Scalar &txtColor, int thickness);

    void clearLinks();

    void createGraph(float graphLinkDistance);

    void createGraphNeighborRelative(float graphNeigborDistanceRelativeLink);

};

#endif // SONARDESCRITOR_H
