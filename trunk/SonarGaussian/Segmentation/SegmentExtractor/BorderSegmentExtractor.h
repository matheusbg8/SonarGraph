#ifndef BORDERSEGMENTEXTRACTOR_H
#define BORDERSEGMENTEXTRACTOR_H

#include "SegmentExtractor.h"

class BorderSegmentExtractor : public SegmentExtractor
{
protected:
    unsigned maxSampleSize,
             searchThreshold;

public:
    BorderSegmentExtractor();

    // SegmentExtractor interface
public:
    void createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col);    
    void load(ConfigLoader &config);
    void setThreshold(unsigned threshold);


};

#endif // BORDERSEGMENTEXTRACTOR_H
