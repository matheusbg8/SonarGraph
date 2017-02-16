#ifndef VMBYEDGELENGTH_H
#define VMBYEDGELENGTH_H

#include "VertexMatcher.h"
#include "Tools/HungarianAlgorithm.h"


class VMbyEdgeLength : public VertexMatcher
{
private:
    HungarianAlgorithm ha;

    void computMatchs();

public:
    VMbyEdgeLength();

    // VertexMatcher interface
public:
    bool load(ConfigLoader &config);

    float vertexMatch(Gaussian &gu, Gaussian &gv,
                      vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                      unsigned *matchEdges);

    float  vertexMatch(Gaussian &gu, Gaussian &gv,
                      vector<GraphLink *> &u, vector<GraphLink *> &v,
                      vector<MatchInfoWeighted> &matchEdges);

};

#endif // VMBYEDGELENGTH_H
