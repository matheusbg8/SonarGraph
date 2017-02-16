#ifndef SEGMENTEXTRACTOR_H
#define SEGMENTEXTRACTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

#include "Segment.h"
#include "Sonar/SonarConfig/ConfigLoader.h"

class Segmentation;

class SegmentExtractor
{
protected:
    Mat *searchMask;
    Segmentation *m_seg;

public:

    SegmentExtractor();
    virtual ~SegmentExtractor();

    void setSegmentation(Segmentation *seg);

// Interface SegmentExtractor
    virtual void createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col) = 0;
    virtual void load(ConfigLoader &config) = 0;
    virtual void setThreshold(unsigned threshold);
    virtual void calibUI(Mat &img16bits);

};

#endif // SEGMENTEXTRACTOR_H
