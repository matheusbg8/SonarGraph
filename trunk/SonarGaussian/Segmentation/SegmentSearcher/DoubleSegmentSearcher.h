#ifndef DOUBLESEGMENTSEARCHER_H
#define DOUBLESEGMENTSEARCHER_H

#include "SegmentSearcher.h"
#include "Sonar/SonarConfig/ConfigLoader.h"

class DoubleSegmentSearcher : public SegmentSearcher
{
protected:
    unsigned rowJump,
             colJump,
             firstRhoLinear,
             secondRhoLinear,
             minSampleSize;

public:
    DoubleSegmentSearcher();

    // SegmentSearcher interface
public:
    void segment(Mat &img16bits, vector<Segment *> *sg);
    void load(ConfigLoader &config);
};

#endif // DOUBLESEGMENTSEARCHER_H
