#ifndef VERTEXMATCHER_H
#define VERTEXMATCHER_H

#include "Sonar/GraphLink.h"

typedef pair<float,float> PFF;
typedef pair<unsigned,unsigned> PUU;
typedef pair<float,PUU> PFPUU;
typedef pair<unsigned,PUU> PUPUU;
typedef pair<PUU,PFF> PUUPFF;


class VertexMatcher
{
public:

    virtual float vertexMatch(vector<GraphLink *> &u, vector<GraphLink *> &v) = 0;
    virtual float vertexMatch(vector<GraphLink *> &u, vector<GraphLink *> &v,
                              vector<PUUPFF> &matchEdges) = 0;
};

#endif // VERTEXMATCHER_H
