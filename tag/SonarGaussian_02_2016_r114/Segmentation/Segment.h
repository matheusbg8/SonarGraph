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

    Mat result; /**< 1 chamenel 3xN matrix , result[0,:] = pixel row found , row[1,:] = pixel col found and result[2,:] = pixel intensity of pixel found */

    unsigned N; /**< Pixel found count */
    unsigned mRow, MRow, mCol, MCol;

    void drawSegment(Mat &bgrImg, const Scalar &color);

    void drawSegmentBox(Mat &bgrImg, const Scalar &color);

    void rotateSegment(float degreeAng, float cx, float cy);

    void merge(Segment *seg);

};

#endif // SEGMENT_H
