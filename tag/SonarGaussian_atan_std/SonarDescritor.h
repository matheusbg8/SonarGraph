#ifndef SONARDESCRITOR_H
#define SONARDESCRITOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<vector>
#include"Gaussian.h"
#include"GraphLink.h"

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

};

#endif // SONARDESCRITOR_H
