#ifndef FULLSEGMENTEXTRACTOR_H
#define FULLSEGMENTEXTRACTOR_H

#include "SegmentExtractor.h"
#include "Sonar/SonarConfig/ConfigLoader.h"

class FullSegmentExtractor : public SegmentExtractor
{
protected:
    unsigned maxSampleSize,
             rhoRecursive;

public:
    FullSegmentExtractor();

    // SegmentExtractor interface
public:
    void createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col);
    void load(ConfigLoader &config);
    void setThreshold(unsigned threshold);

};

#endif // FULLSEGMENTEXTRACTOR_H
