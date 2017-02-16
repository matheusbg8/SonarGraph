#ifndef GMFBESTDIRECT_H
#define GMFBESTDIRECT_H

#include "GraphMatchFinder.h"

class GMFBestDirect : public GraphMatchFinder
{
private:
    unsigned minEdgeToMatch;

    SonarDescritor *current_sd1,
                   *current_sd2;

    bool findBest(unsigned uId, unsigned &vId);
    void fastMatchCheck(vector<MatchInfo> &vertexMatch);
    void fastMatchCheck2(vector<MatchInfo> &vertexMatch);

public:
    GMFBestDirect();

    // GraphMatchFinder interface
public:
    bool load(ConfigLoader &config);
    void findMatch(SonarDescritor *sd1, SonarDescritor *sd2, vector<MatchInfo> &vertexMatch);
    void findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2, vector<MatchInfoExtended> &matchInfo);
};

#endif // GMFBESTDIRECT_H
