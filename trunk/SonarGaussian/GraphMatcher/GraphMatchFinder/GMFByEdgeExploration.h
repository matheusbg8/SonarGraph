#ifndef GMFBYEDGEEXPLORATION_H
#define GMFBYEDGEEXPLORATION_H

#include "GraphMatchFinder.h"

class GMFByEdgeExploration : public GraphMatchFinder
{
private:
    unsigned minEdgeToMatch,
             minEdgeDiffToAcceptMatch;

    SonarDescritor *current_sd1,
                   *current_sd2;

    unsigned lbCount;

    vector<int> lb1, lb2;

    bool findBest(unsigned uId, unsigned &vId);
    bool explore(unsigned uId, unsigned vId, vector<MatchInfoWeighted> &vertexMatch);

    bool fastMatchCheck(vector<MatchInfoWeighted> &vertexMatch);
    bool fastMatchCheck2(vector<MatchInfoWeighted> &vertexMatch);

public:
    GMFByEdgeExploration();

    // GraphMatchFinder interface
public:
    bool load(ConfigLoader &config);
    void findMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                   vector<MatchInfo> &vertexMatch);
    void findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                        vector<MatchInfoExtended> &matchInfo);
};

#endif // GMFBYEDGEEXPLORATION_H
