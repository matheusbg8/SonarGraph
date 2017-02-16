#include "MatchInfoExtended.h"

MatchInfoExtended::MatchInfoExtended(int uID, int vID,
                                     float bestScore, float sBestScore):
        MatchInfo(uID,vID),
    bestScore(bestScore), sBestScore(sBestScore)
{

}
