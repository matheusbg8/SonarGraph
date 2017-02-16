#include "VertexMatcher.h"

#ifndef VMBRUTALFORCE_H
#define VMBRUTALFORCE_H


/**
 * @brief This method compute the vertex affinity
 * witout use shape characteristic of features. This
 * is a brutal force aproach that have to find the best
 * soluction but its computacional complexti is exponencial
 * and impratical to be used.
 *
 * This class implementation was not finished yet.
 *
 */
class VMBrutalForce : public VertexMatcher
{
public:
    typedef pair<unsigned,unsigned> PUU;
    typedef pair<float, PUU> PFPUU;
    typedef pair<unsigned, PUU> PUPUU;

    VMBrutalForce();


    void brutalFORCExplore(vector<GraphLink *> &u,
                           vector< vector< vector<PFPUU> > > &explore);

    void brutalFORCEEdgeConfigCompare(vector<GraphLink *> &u, vector<PFPUU> &uExp,
                                      vector<GraphLink *> &v, vector<PFPUU> &vExp
                                     );

    // VertexMatcher interface
public:
    bool load(ConfigLoader &config);
    float vertexMatch(Gaussian &gu, Gaussian &gv, vector<GraphLink *> &eu, vector<GraphLink *> &ev, unsigned *matchEdges);
    float vertexMatch(Gaussian &gu, Gaussian &gv, vector<GraphLink *> &u, vector<GraphLink *> &v, vector<MatchInfoWeighted> &matchEdges);
};

#endif // VMBRUTALFORCE_H
