#ifndef SONAR_H
#define SONAR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

#include "Enuns.h"
#include "SonarConfig/ConfigLoader.h"
#include "GraphLink.h"
#include "SonarDescritor.h"
#include "GraphMatcher/GraphMatcher.h"
#include "Cronometer.h"

#include "Segmentation/Segmentation.h"

using namespace std;
using namespace cv;

// ======  Debug section =====
// #define SONAR_DEBUG
// ==== end debug section ====

class Sonar
{
private:
    Cronometer cronometer; // To measure the execution time

    Segmentation segmentation;
public:
    GraphMatcher matcher;
private:

    float stdDevMultiply; // Number of times we multiply stdDev to work
    float graphLinkDistance; // Distance to link two vertex
    float graphNeigborDistanceRelativeLink; // Distance to link two vertex
    GraphCreatorMode graphCreatorMode;

    Mat img16bits;
    Mat colorImg;

    vector<SonarDescritor*> descriptors;

    vector<Mat> storedImgs;


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

    bool deleteDescriptors;

//private:

    void segmentationCalibUI(Mat &img16b);

    void createGaussian(Mat &img, SonarDescritor *sd, bool doMerge=false);
    void createGaussian(Mat &img, vector<Gaussian> &gs, bool doMerge=false);

    void drawGaussians(Mat colorImg, SonarDescritor *sd);

    bool intersec(Gaussian &a, Gaussian &b);

    void mergeGaussian(SonarDescritor *sd, unsigned a, unsigned b);

    void pseudoMergeGaussian(SonarDescritor *sd, unsigned a, unsigned b);

    void createGraph(SonarDescritor *sd);
    void createGraphNeighborRelative(SonarDescritor *sd);

public:
    Sonar(ConfigLoader &config, bool deleteDescriptors=true);
    Sonar(bool deleteDescriptors=true);
    ~Sonar();

    void loadConfig(ConfigLoader &config);

    SonarDescritor* newImage(Mat img);

    SonarDescritor* newImageDebug(Mat &imgGray16bits, Mat &imgResult);

    SonarDescritor* newImageDirect(Mat &img);

    void computeMatchs(SonarDescritor *sd1, SonarDescritor *sd2,
                       vector<MatchInfo> &matchs);

    float computeVertexMatch(SonarDescritor *sd1, unsigned vertexId1,
                            SonarDescritor *sd2, unsigned vertexId2,
                            vector<MatchInfoWeighted> &matchs);

    float computeVertexMatch(Gaussian &gu, Gaussian &gv,
                            vector<GraphLink *> &u, vector<GraphLink *> &v,
                            vector<MatchInfoWeighted> &matchEdges);

    void setStoreImgs(bool store);

    void clearDescriptors();
};

#endif // SONAR_H
