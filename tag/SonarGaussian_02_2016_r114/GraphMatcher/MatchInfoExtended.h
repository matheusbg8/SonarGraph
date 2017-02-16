#ifndef MATCHINFOEXTENDED_H
#define MATCHINFOEXTENDED_H

#include<vector>
#include "GraphMatcher/MatchInfo.h"

using namespace std;

class MatchInfoExtended: public MatchInfo
{
public:
    typedef pair<float,float> PFF;
    typedef pair<unsigned,unsigned> PUU;
    typedef pair<float,PUU> PFPUU;
    typedef pair<PUU,PFF> PUUPFF;

    float bestScore, sBestScore;

    // Best match informations
    // < (uEdgeID, vEdgeID), (u-v_distanceDiff, u-v_SlopeDiff)
    vector<PUUPFF> edgeMatchInfo;

    MatchInfoExtended(int uID=-1, int vID=-1,
                      float bestScore=-1.f, float sBestScore=-1.f);
};

#endif // MATCHINFOEXTENDED_H
