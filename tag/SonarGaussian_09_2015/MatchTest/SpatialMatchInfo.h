#ifndef SPATIALMATCHINFO_H
#define SPATIALMATCHINFO_H

#include<vector>

using namespace std;

class SpatialMatchInfo
{
public:
    typedef pair<float,float> PFF;
    typedef pair<unsigned,unsigned> PUU;
    typedef pair<PFF,PUU> PUUPFF;

    // Match informations
    int uID, vID;
    float bestScore, sBestScore;

    // Best match informations
    // < (uEdgeID, vEdgeID), (u-v_distanceDiff, u-v_SlopeDiff)
    vector<PUUPFF> edgeMatchInfo;


    SpatialMatchInfo();
};

#endif // SPATIALMATCHINFO_H
