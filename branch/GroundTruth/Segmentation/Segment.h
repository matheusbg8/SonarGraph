#ifndef SEGMENT_H
#define SEGMENT_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

using namespace std;
using namespace cv;

class Segment
{
public:

    Mat result; // 1 chamenel 3xN matrix , row 0 = row found , row 1 = col and row 2 = pixel intensity of pixel found
    unsigned N; // Pixel found count
    unsigned mRow, MRow, mCol, MCol;

    /* Draw pixels of segment on image BGR */
    void drawSegment(Mat &bgrImg, const Scalar &color);

    /* Draw box of segment on image BGR */
    void drawSegmentBox(Mat &bgrImg, const Scalar &color);

    /* Rotate all pixel degreeAng around cx, cy position */
    void rotateSegment(float degreeAng, float cx, float cy);

};

#endif // SEGMENT_H
