#ifndef GRAPHMATCHFINDER_H
#define GRAPHMATCHFINDER_H

#include "Sonar/SonarDescritor.h"
#include "Sonar/SonarConfig/ConfigLoader.h"

#include "GraphMatcher/VertexMatcher/VertexMatcher.h"
#include "GraphMatcher/MatchInfo/MatchInfo.h"
#include "GraphMatcher/MatchInfo/MatchInfo.h"

/**
 * @brief This class compute the error between two graphs,
 * it compute error between vertexes in diferent ways
 * using a VertexMatcher and decides which pairs of vertex match.
 *
 */
class GraphMatchFinder
{
protected:
    VertexMatcher *m_vertexMatcher;
public:
    GraphMatchFinder();

    void setVertexMatcher(VertexMatcher *vertexMatcher);

//Interface
    virtual bool load(ConfigLoader &config) =0;

    virtual void findMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                       vector<MatchInfo> &vertexMatch) =0;

    virtual void findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                            vector<MatchInfoExtended> &matchInfo) =0;
};

#endif // GRAPHMATCHFINDER_H
