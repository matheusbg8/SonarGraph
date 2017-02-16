#ifndef MOSAIC_H
#define MOSAIC_H

#include "GroundTruth/MatchHandler.h"

class Mosaic
{

public:
    // HGraph[srcFrame] = (destFrame, Homgraphy)
    vector<pair<unsigned, Mat> > HGraph;

    Mosaic();

    void createMosaic();


    void createHomographyGraph(vector<FrameGT> &frames);

};

#endif // MOSAIC_H
