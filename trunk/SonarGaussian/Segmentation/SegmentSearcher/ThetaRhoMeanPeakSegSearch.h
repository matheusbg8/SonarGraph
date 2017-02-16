#ifndef THETARHOMEANPEAKSEGSEARCH_H
#define THETARHOMEANPEAKSEGSEARCH_H

#include "SegmentSearcher.h"
#include "Tools/CircularQueue.h"

/* ==== Debub section ===== */
#define THETARHOMEANPEAKSEGSEARCH_DRWING_DEBUG
//#define CALIB_THETARHOMEANPEAKSEGSEARCH_DRWING_DEBUG
/* ==== END Debub section ===== */

class ThetaRhoMeanPeakSegSearch : public SegmentSearcher
{
private:
    int nBeams,
        startBin,
        Hmin,  /**< Minimum height to a peak be acceptable $H_{min}$ */
        minSampleSize,
        meanWindowSize;

    int sonVerticalPosition;

    float bearing;

public:
    ThetaRhoMeanPeakSegSearch();


    // SegmentSearcher interface
public:
    void segment(Mat &img16bits, vector<Segment *> *sg);
    void load(ConfigLoader &config);
    void calibUI(Mat &img16bits);
};

#endif // THETARHOMEANPEAKSEGSEARCH_H
