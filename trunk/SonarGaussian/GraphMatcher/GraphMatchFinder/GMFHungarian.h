#ifndef GMFHUNGARIAN_H
#define GMFHUNGARIAN_H

#include "GraphMatchFinder.h"
#include "Tools/HungarianAlgorithm.h"
#include "Tools/IndexMapping.h"

class GMFHungarian : public GraphMatchFinder
{
private:
    unsigned minMatches;
    HungarianAlgorithm hu;

    IndexMapping map1, map2;


public:
    GMFHungarian();

    // GraphMatchFinder interface
public:
    bool load(ConfigLoader &config);
    void findMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                   vector<MatchInfo> &vertexMatch);

    void findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                        vector<MatchInfoExtended> &matchInfo);
};

#endif // GMFHUNGARIAN_H
