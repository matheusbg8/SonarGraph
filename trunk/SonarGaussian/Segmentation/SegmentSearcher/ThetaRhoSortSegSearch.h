#ifndef THETARHOSORTSEGSEARCH_H
#define THETARHOSORTSEGSEARCH_H

#include "SegmentSearcher.h"
#include "Sonar/SonarConfig/ConfigLoader.h"

// ==== DEBUG SECTION ====
#define SEGMENTATION_DRWING_DEBUG
// ==== END DEBUG SECTION ====

class ThetaRhoSortSegSearch : public SegmentSearcher
{
private:
    unsigned nBeams,
             startBin,
             Hmin,  /**< Minimum height to a peak be acceptable $H_{min}$ */
             minSampleSize;

    int sonVerticalPosition;

    float bearing,
          PiEnd, /**< If a peak down low tham resetTax of height, reset a peak. */
          PiRecursive;  /**< Height percent of acceptable peak used for threshold */

public:
    ThetaRhoSortSegSearch();

    // SegmentSearcher interface
public:
    void segment(Mat &img16bits, vector<Segment *> *sg);
    void load(ConfigLoader &config);
    void calibUI(Mat &img16bits);
};

#endif // THETARHOSORTSEGSEARCH_H
