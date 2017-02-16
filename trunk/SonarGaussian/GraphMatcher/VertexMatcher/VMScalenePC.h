#include "VertexMatcher.h"

#ifndef VMSCALENEPC_H
#define VMSCALENEPC_H

class VMScalenePC : public VertexMatcher
{
protected:
    float epsMax,  /**< Max edge error allowed */
          gaussianRatioW;  /**< It's the weight of stdX/stdY gaussian aways between 0 and 1*/

public:
    VMScalenePC();

    // VertexMatcher interface
public:

    bool load(ConfigLoader &config);

    float vertexMatch(Gaussian &gu, Gaussian &gv,
                      vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                      unsigned *matchEdges=0x0);

    float vertexMatch(Gaussian &gu, Gaussian &gv,
                      vector<GraphLink *> &u, vector<GraphLink *> &v,
                      vector<MatchInfoWeighted> &matchEdges);

};

#endif // VMSCALENEPC_H
