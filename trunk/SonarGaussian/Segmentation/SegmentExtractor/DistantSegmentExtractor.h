#ifndef DISTANTSEGMENTEXTRACTOR_H
#define DISTANTSEGMENTEXTRACTOR_H

#include "SegmentExtractor.h"
#include "Sonar/SonarConfig/ConfigLoader.h"

class DistantSegmentExtractor : public SegmentExtractor
{
protected:
    unsigned maxSampleSize,
             rhoRecursive;

    int searchDistance;

public:
    DistantSegmentExtractor();

    // SegmentExtractor interface
public:
    void createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col);
    void load(ConfigLoader &config);
    void setThreshold(unsigned threshold);

};

#endif // DISTANTSEGMENTEXTRACTOR_H
