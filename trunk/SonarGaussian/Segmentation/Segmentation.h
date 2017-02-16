#ifndef SEGMENTATION_H
#define SEGMENTATION_H

// 31- 245

// =====   DEBUG ====
//#define SEGMENTATION_MEMORORY_DEBUG
//#define SEGMENTATION_DRWING_DEBUG
//#define SEGMENTATION_SEARCH_DEBUG
//#define SEGMENTATION_EXECUTION_TIME_DEBUG
// ===== END DEBUG ===

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include "Segment.h"
#include "SegmentSearcher/SegmentSearcher.h"
#include "SegmentExtractor/SegmentExtractor.h"

using namespace std;
using namespace cv;

#include "Sonar/SonarConfig/ConfigLoader.h"

class Segmentation
{
public:
/* === DEBUG atributes ==== */
 #ifdef SEGMENTATION_SEARCH_DEBUG
    //    Mat imgDebug(img16bits.rows, img16bits.cols,CV_8UC3,Scalar(0,0,0));
        Mat imgDebug;
  #endif
 /* === DEBUG atributes ==== */

    ushort pixelThreshold; // Threshold for pixel search (start floodfill) on 8bits normalized image
    ushort searchThreshold; // Threshold to floodfill search on 16bits image
    unsigned minSampleSize; // Minimun amount of pixel to create a vertex
    unsigned maxSampleSize;
    int searchDistance;

    Mat searchMask; // Used by floodfill search
    Mat imgMask;

    SegmentSearcher *m_segSearcher;
    SegmentExtractor *m_segExtractor;

    vector<Segment*> segments;

    static const int neighborRow[8];
    static const int neighborCol[8];

    void reduceToConvexHull(Segment *seg);

    unsigned neighborSum(Mat &grayImg, unsigned row, unsigned col);

    void config();

    /* Return a segment with ID */
    Segment* segment(unsigned id);

    void resetMask(unsigned rows, unsigned cols);
// public:

    Segmentation();
    Segmentation(ConfigLoader &config);
    ~Segmentation();

    void load(ConfigLoader &config);
    void loadDefaultConfig();

    void setSegmentExtractor(SegmentExtractor* segExtractor);
    void setSegmentSearcher(SegmentSearcher* segSearcher);


    void segment(Mat &img16bits,vector<Segment *> *sg);
    void createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col);


    void interativeCalibUI(Mat &img16bits);

/// The methods below are used just for tests

    void createSegmentImg(Mat &img16bits);

    void createLogImg(Mat &img16bits);

    bool createGradientImg(Mat &img16bits);

    void createRTImage(Mat &img16bits);

    bool createImg8bitsTruncated(Mat &img16bits);

    void createAdaptativeThreshold(Mat &img16bits);

    void createAdaptativeThresholdOtsu(Mat &img16bits);

    void createAdaptativeThreshold2(Mat &img16bits);

    void createRTPlot(Mat &img16bits);

    void interativeRTPlot(Mat &img16bits);

    void extractLine(Mat &result, Point2f src, Point2f dest, Scalar &color);

    void extractLine2(Mat &result, Point2f src, float radAng, unsigned lenght, Scalar &color);

    void morphologicalOperator();

};

#endif // SEGMENTATION_H
