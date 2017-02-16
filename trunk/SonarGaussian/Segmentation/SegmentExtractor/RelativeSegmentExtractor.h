#ifndef RELATIVESEGMENTEXTRACTOR_H
#define RELATIVESEGMENTEXTRACTOR_H

#include "SegmentExtractor.h"

class RelativeSegmentExtractor : public SegmentExtractor
{
private:
    unsigned decaiment;
    unsigned maxSampleSize;

public:
    RelativeSegmentExtractor();

    // SegmentExtractor interface
public:
    void createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col);
    void load(ConfigLoader &config);
    void setThreshold(unsigned threshold);

};

#endif // RELATIVESEGMENTEXTRACTOR_H
