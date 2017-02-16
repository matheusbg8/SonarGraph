#ifndef MATCHINFOEXTENDED_H
#define MATCHINFOEXTENDED_H

#include<vector>
#include "GraphMatcher/MatchInfo/MatchInfo.h"
#include "GraphMatcher/MatchInfo/MatchInfoWeighted.h"

using namespace std;

class MatchInfoExtended: public MatchInfo
{
public:

    float bestScore, sBestScore;

    // Best match informations
    // < (uEdgeID, vEdgeID), (u-v_distanceDiff, u-v_SlopeDiff)
    vector<MatchInfoWeighted> edgeMatchInfo;

    MatchInfoExtended(int uID=-1, int vID=-1,
                      float bestScore=-1.f, float sBestScore=-1.f);
};

#endif // MATCHINFOEXTENDED_H
