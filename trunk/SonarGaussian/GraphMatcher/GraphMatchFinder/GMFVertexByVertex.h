#include "GraphMatchFinder.h"

#ifndef GMFVERTEXBYVERTEX_H
#define GMFVERTEXBYVERTEX_H

class GMFVertexByVertex : public GraphMatchFinder
{
protected:
    float meaningfulness;
    unsigned minSimilarEdgeToMatch;

public:
    GMFVertexByVertex();

    // GraphMatchFinder interface
public:
    bool load(ConfigLoader &config);

    void findMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                   vector<MatchInfo> &vertexMatch);

    void findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                        vector<MatchInfoExtended> &matchInfo);
};

#endif // GMFVERTEXBYVERTEX_H
