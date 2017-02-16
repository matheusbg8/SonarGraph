#ifndef SEGMENTATION_H
#define SEGMENTATION_H

// 31- 245

// =====   DEBUG ====
//#define SEGMENTATION_MEMORORY_DEBUG
#define SEGMENTATION_DRWING_DEBUG
//#define SEGMENTATION_SEARCH_DEBUG
//#define SEGMENTATION_EXECUTION_TIME_DEBUG

// ===== END DEBUG ===

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include "Segment.h"

using namespace std;
using namespace cv;

class Segmentation
{
public:
/* === DEBUG atributes ==== */
 #ifdef SEGMENTATION_SEARCH_DEBUG
    //    Mat imgDebug(img16bits.rows, img16bits.cols,CV_8UC3,Scalar(0,0,0));
        Mat imgDebug;
  #endif
 /* === DEBUG atributes ==== */

    static const int neighborRow[8];
    static const int neighborCol[8];

    bool computeHistogranThreshold;
    bool computeMaxPixel;
    unsigned maxPixel;

    ushort pixelThreshold; // Threshold for pixel search (start floodfill) on 8bits normalized image
    ushort searchThreshold; // Threshold to floodfill search on 16bits image
    unsigned minSampleSize; // Minimun amount of pixel to create a vertex
    unsigned maxSampleSize;
    int searchDistance;

    Mat searchMask; // Used by floodfill search
    Mat imgMask;
    vector<Segment*> segments;

    typedef pair<unsigned,unsigned> PUU;
    typedef pair<unsigned,PUU> PUPUU;
    vector<PUPUU> peaks; // Used for segmentWithPeaks

    unsigned peakThreshold;
    unsigned maxPeakSize;
    unsigned acceptPeakHeight;

    /* Search for neighbord pixels with intensity more than searchThreshold */
    void createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col);

    /* Search for neighbord pixels with intensity more than searchThreshold */
    void createBorderSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col, unsigned searchThreshold);

    void createSegmentDist(Segment *seg, Mat img16bits, unsigned row, unsigned col);

    /* Threshold auto */
    void serachAuto(Segment *seg, Mat img16bits, unsigned row, unsigned col);

    /* Search for neighbord pixels with intensity inside a range of intensity relative to started pixel intensity*/
    void createSegmentRelative(Segment *seg, Mat img16bits, unsigned row, unsigned col, unsigned deacaiment);

    /* Search for neighbord pixels with intensity inside a range of intensity relative to started pixel intensity*/
    void createSegmentRelativePixelCount(Segment *seg, Mat img16bits, unsigned row, unsigned col, unsigned pixelCount);

    void reduceToConvexHull(Segment *seg);

    bool isPeak(Mat &img16bits, unsigned row, unsigned col);
    Scalar peakColor(Mat &img16bits, unsigned row, unsigned col);

    unsigned neighborSum(Mat &grayImg, unsigned row, unsigned col);

    /* Return a segment with ID */
    Segment* segment(unsigned id);

    void resetMask(unsigned rows, unsigned cols);
// public:

    Segmentation();
    ~Segmentation();

    /* Create the segments of img16bits on sg vector, you should not delete this vector never!
     *  and this segments will be valid until new call of this method
    */
    void segment(Mat &img16bits, vector<Segment *> *sg);

    void segmentDouble(Mat &img16bits, vector<Segment *> *sg);

    void segmentWithPeaks(Mat &img16bits, vector<Segment *> *sg);

    void segmentWithTR(Mat &img16bits, vector<Segment *> *sg);

    void createSegmentImg(Mat &img16bits);

    void createLogImg(Mat &img16bits);

    bool createGradientImg(Mat &img16bits);

    bool createPeakImg(Mat &img16bits);

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
