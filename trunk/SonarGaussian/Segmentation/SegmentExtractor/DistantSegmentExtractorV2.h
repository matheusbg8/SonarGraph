#ifndef DISTANTSEGMENTEXTRACTORV2_H
#define DISTANTSEGMENTEXTRACTORV2_H

#include "SegmentExtractor.h"
#include "Sonar/SonarConfig/ConfigLoader.h"

class DistantSegmentExtractorV2 : public SegmentExtractor
{
protected:
    unsigned maxSampleSize,
             rhoRecursive;

    int searchDistance;
public:
    DistantSegmentExtractorV2();

    void get3Neighbor(int id, unsigned refRow, unsigned refCol,
                      unsigned *lastRow, unsigned *lastCol,
                      unsigned *currentRow, unsigned *currentCol,
                      unsigned *nextRow, unsigned *nextCol);

    void findContours();

    // SegmentExtractor interface
public:
    void createSegment(Segment *seg, Mat img16bits, unsigned row, unsigned col);
    void load(ConfigLoader &config);
    void setThreshold(unsigned threshold);
};

#endif // DISTANTSEGMENTEXTRACTORV2_H
