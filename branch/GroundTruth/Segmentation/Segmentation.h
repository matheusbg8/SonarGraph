#ifndef SEGMENTATION_H
#define SEGMENTATION_H


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
    static const int neighborRow[8];
    static const int neighborCol[8];

    bool computeHistogranThreshold;
    bool computeMaxPixel;
    unsigned maxPixel;

    ushort pixelThreshold; // Threshold for pixel search (start floodfill) on 8bits normalized image
    ushort searchThreshold; // Threshold to floodfill search on 16bits image
    unsigned minSampleSize; // Minimun amount of pixel to create a vertex
    unsigned maxSampleSize;
    unsigned searchDistance;

    Mat searchMask; // Used by floodfill search
    vector<Segment*> segments;

    /* Search for neighbord pixels with intensity more than searchThreshold */
    void createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col);

    void createSegmentDist(Segment *seg, Mat img16bits, unsigned row, unsigned col);

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

};

#endif // SEGMENTATION_H
