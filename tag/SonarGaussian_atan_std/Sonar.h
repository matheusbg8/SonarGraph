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
#include "GraphMatcher.h"
#include "Cronometer.h"

using namespace std;
using namespace cv;



class Sonar
{
private:

    Cronometer cronometer; // To measure the execution time

    static const int neighborRow[8];
    static const int neighborCol[8];

    Mat floodFillResult; // 2 channels 1xN matrix , channel 0 = row found , channel 2 = col found by floodfill search
    unsigned floodFillResultCount; // Pixel found count by floodfill search
    unsigned MRCol, MRow, MCRow, MCol;

    bool computeHistogranThreshold;
    bool computeMaxPixel;
    unsigned maxPixel;

    float stdDevMultiply; // Number of times we multiply stdDev to work
    float graphLinkDistance; // Distance to link two vertex
    float graphNeigborDistanceRelativeLink; // Distance to link two vertex
    GraphCreatorMode graphCreatorMode;


    ushort pixelThreshold; // Threshold for pixel search (start floodfill) on 8bits normalized image
    ushort searchThreshold; // Threshold to floodfill search on 16bits image
    unsigned minSampleSize; // Minimun amount of pixel to create a vertex
    unsigned maxSampleSize;

    Mat searchMask; // Used by floodfill search
    Mat colorImg;
    Mat img16bits;
    Mat img8bits;

    vector<SonarDescritor*> descriptors;
    GraphMatcher matcher;


    bool drawPixelFound;
    bool drawElipses;
    bool drawElipsesAngle;
    bool drawEdges;
    bool drawEdgesAngle;
    bool drawEdgesInvAngle;
    bool drawVertexID;
    bool drawStdDevValue;

    bool saveGraphImg;
    bool saveTruncateImg;
    bool saveMatchImgs;

    void floodFill(unsigned row, unsigned col);

    float calcGaussianAng(float x, float y,float dx, float dy,float Mx, float MXy , float My, float MYx);

    void calculateMeanAndStdDerivation(Mat &img, int row, int col, float &rowMean, float &colMean, float &pixelMean, float &rowStdDev, float &colStdDev, float &pixelStdDev, float &ang, unsigned &N);

    void drawResultPixels(Mat &img, Scalar color);

    void rotateResult(unsigned N, float angDegree, float ox, float oy);

    void createGaussian(Mat &img, SonarDescritor *sd);

    void drawGaussians(Mat colorImg, SonarDescritor *sd);

    bool intersec(Gaussian &a, Gaussian &b);

    void mergeGaussian(SonarDescritor *sd, unsigned a, unsigned b);

    void pseudoMergeGaussian(SonarDescritor *sd, unsigned a, unsigned b);

    void createGraph(SonarDescritor *sd);
    void createGraphNeighborRelative(SonarDescritor *sd);

public:
    Sonar(const SonarConfig &config = SonarConfig());

    SonarDescritor* newImage(Mat img);

};

#endif // SONAR_H
