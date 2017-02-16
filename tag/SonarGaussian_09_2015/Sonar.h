#ifndef SONAR_H
#define SONAR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#include "Enuns.h"
#include "SonarConfig.h"
#include "GraphLink.h"
#include "SonarDescritor.h"
#include "GraphMatcher/GraphMatcher.h"
#include "Cronometer.h"

#include "Segmentation/Segmentation.h"

using namespace std;
using namespace cv;

class Sonar
{
private:
    Cronometer cronometer; // To measure the execution time

    Segmentation segmentation;

    float stdDevMultiply; // Number of times we multiply stdDev to work
    float graphLinkDistance; // Distance to link two vertex
    float graphNeigborDistanceRelativeLink; // Distance to link two vertex
    GraphCreatorMode graphCreatorMode;

    Mat img16bits;
    Mat colorImg;

    vector<SonarDescritor*> descriptors;

    vector<Mat> storedImgs;

    GraphMatcher matcher;

public:
    bool drawPixelFound;
    bool drawElipses;
    bool drawElipsesAngle;
    bool drawEdges;
    bool drawEdgesAngle;
    bool drawEdgesInvAngle;
    bool drawVertexID;
    bool drawStdDevValue;
    bool drawEachVetex;

    bool storeImgs;

    bool saveGraphImg;
    bool saveTruncateImg;
    bool saveMatchImgs;
private:

    void createGaussian(Mat &img, SonarDescritor *sd);

    void drawGaussians(Mat colorImg, SonarDescritor *sd);

    bool intersec(Gaussian &a, Gaussian &b);

    void mergeGaussian(SonarDescritor *sd, unsigned a, unsigned b);

    void pseudoMergeGaussian(SonarDescritor *sd, unsigned a, unsigned b);

    void createGraph(SonarDescritor *sd);
    void createGraphNeighborRelative(SonarDescritor *sd);

public:
    Sonar(const SonarConfig &config = SonarConfig());
    ~Sonar();

    SonarDescritor* newImage(Mat img);

    SonarDescritor* newImageDebug(Mat &imgGray16bits, Mat &imgResult);

    SonarDescritor* newImageDirect(Mat &img);

    void clearDescriptors();
};

#endif // SONAR_H
