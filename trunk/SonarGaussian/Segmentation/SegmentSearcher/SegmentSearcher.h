#ifndef SEGMENTSEARCHER_H
#define SEGMENTSEARCHER_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Segment.h"
#include <vector>

using namespace cv;
using namespace std;

#include "SegmentExtractor/SegmentExtractor.h"
#include "Sonar/SonarConfig/ConfigLoader.h"

class Segmentation;

class SegmentSearcher
{
protected:
    Mat *searchMask; /**< Indicate extracted pixels, used by segment extraction process*/
    Mat *imgMask; /**< Indicate the pixels how will be process */
    Segmentation *m_seg;
    SegmentExtractor *m_extractor;

public:
    SegmentSearcher();
    virtual ~SegmentSearcher();

    void setExtractor(SegmentExtractor *extractor);
    void setSegmentation(Segmentation *seg);

// Interface SegmentSearcher
    virtual void segment(Mat &img16bits,vector<Segment *> *sg) = 0;
    virtual void load(ConfigLoader &config) = 0;
    virtual void calibUI(Mat &img16bits);
};

#endif // SEGMENTSEARCHER_H
