#ifndef MATCHINFOWEIGHTED_H
#define MATCHINFOWEIGHTED_H

#include "GraphMatcher/MatchInfo/MatchInfo.h"

class MatchInfoWeighted : public MatchInfo
{
public:
    float score;

    MatchInfoWeighted(int uID=-1, int vID=-1, float score=-1.f);
};

#endif // MATCHINFOWEIGHTED_H
