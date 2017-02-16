#ifndef GMFBYVERTEXMATCH_H
#define GMFBYVERTEXMATCH_H

#include "GraphMatchFinder.h"

class GMFByVertexMatch : public GraphMatchFinder
{
private:
    unsigned minEdgeToMatch,
             minEdgeDiffToAcceptMatch;

    void fastMatchCheck(SonarDescritor *sd1, SonarDescritor *sd2,
                        vector<MatchInfo> &vertexMatch, unsigned basedMatchId,
                        vector<MatchInfo> &result);

    void fastMatchCheck2(SonarDescritor *sd1, SonarDescritor *sd2,
                         vector<MatchInfo> &vertexMatch, unsigned basedMatchId, vector<MatchInfo> &result);

public:
    GMFByVertexMatch();

    // GraphMatchFinder interface
public:
    bool load(ConfigLoader &config);
    void findMatch(SonarDescritor *sd1, SonarDescritor *sd2, vector<MatchInfo> &vertexMatch);
    void findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2, vector<MatchInfoExtended> &matchInfo);

};

#endif // GMFBYVERTEXMATCH_H
