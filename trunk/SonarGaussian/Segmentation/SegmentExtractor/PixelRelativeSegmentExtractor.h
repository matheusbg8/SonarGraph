#ifndef PIXELRELATIVESEGMENTEXTRACTOR_H
#define PIXELRELATIVESEGMENTEXTRACTOR_H

#include "SegmentExtractor.h"

class PixelRelativeSegmentExtractor : public SegmentExtractor
{
private:
    unsigned maxSampleSize;

public:
    PixelRelativeSegmentExtractor();

    // SegmentExtractor interface
public:
    void createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col);
    void load(ConfigLoader &config);
    void setThreshold(unsigned threshold);

};

#endif // PIXELRELATIVESEGMENTEXTRACTOR_H
