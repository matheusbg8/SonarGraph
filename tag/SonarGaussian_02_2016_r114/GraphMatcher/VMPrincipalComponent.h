#ifndef VMPRINCIPALCOMPONENT_H
#define VMPRINCIPALCOMPONENT_H

#include "GraphMatcher/VertexMatcher.h"

class VMPrincipalComponent : public VertexMatcher
{
public:
    VMPrincipalComponent();

    // VertexMatcher interface
public:
    float vertexMatch(vector<GraphLink *> &u, vector<GraphLink *> &v);

    float vertexMatch(vector<GraphLink *> &u, vector<GraphLink *> &v, vector<PUUPFF> &matchEdges);
};

#endif // VMPRINCIPALCOMPONENT_H
