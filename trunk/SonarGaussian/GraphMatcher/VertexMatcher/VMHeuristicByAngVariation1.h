#ifndef VMHEURISTICBYANGVARIATION1_H
#define VMHEURISTICBYANGVARIATION1_H

#include "VertexMatcher.h"


/**
 * @brief This class was the fist solution to compute
 * vertex affinite. This is a greedy solution and
 * for this reason its not found the best solution
 *
 * Actualy its class doesn't works, because its
 * wasn't adapted to the new class architecture
 * of the project.
 */
class VMHeuristicByAngVariation1 : public VertexMatcher
{
public:
    VMHeuristicByAngVariation1();

    // VertexMatcher interface
public:
    bool load(ConfigLoader &config);
    float vertexMatch(Gaussian &gu, Gaussian &gv,
                      vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                      unsigned *matchEdges);

    float vertexMatch(Gaussian &gu, Gaussian &gv,
                      vector<GraphLink *> &u, vector<GraphLink *> &v,
                      vector<MatchInfoWeighted> &matchEdges);
};

#endif // VMHEURISTICBYANGVARIATION1_H
