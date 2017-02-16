#ifndef SEGMENTWITHPEAK_H
#define SEGMENTWITHPEAK_H

#include "Sonar/SonarConfig/ConfigLoader.h"

#include "SegmentSearcher.h"

class SegmentWithPeak : public SegmentSearcher
{
protected:

    typedef pair<unsigned,unsigned> PUU;
    typedef pair<unsigned,PUU> PUPUU;
    vector<PUPUU> peaks; // Used for segmentWithPeaks

    unsigned rowJump,
             colJump,
             rhoLinear,
             minSampleSize;

public:
    SegmentWithPeak();

    Scalar peakColor(Mat &img16bits, unsigned row, unsigned col);

    bool isPeak(Mat &img16bits, unsigned row, unsigned col);

    bool createPeakImg(Mat &img16bits);

    // SegmentSearcher interface
public:
    void segment(Mat &img16bits, vector<Segment *> *sg);
    void load(ConfigLoader &config);

};

#endif // SEGMENTWITHPEAK_H
