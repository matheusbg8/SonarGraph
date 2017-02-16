#ifndef LINEARSEGMENTSEARCHER_H
#define LINEARSEGMENTSEARCHER_H

#include "SegmentSearcher.h"
#include "Sonar/SonarConfig/ConfigLoader.h"

class LinearSegmentSearcher : public SegmentSearcher
{
protected:
    unsigned rowJump,
             colJump,
             rhoLinear,
             minSampleSize;

public:
    LinearSegmentSearcher();

    // SegmentSearcher interface
public:
    void segment(Mat &img16bits, vector<Segment *> *sg);
    void load(ConfigLoader &config);
};

#endif // LINEARSEGMENTSEARCHER_H
