#include "VertexMatcher.h"

#include "GraphMatcher/MatchInfo/MatchInfoExtended.h"

#ifndef VMHEURISTICBYANGVARIATION_H
#define VMHEURISTICBYANGVARIATION_H


/**
 * @brief This class compute the vertex affinits
 * based on the last heuristic approach, trying
 * to fix the problem. This solution still being
 * a greedy solution and having problens.
 *
 * The main characterist of this solution is that
 * the shape of features are not used.
 *
 *  Implementation OK , but not tested yet
 */
class VMHeuristicByAngVariation : public VertexMatcher
{
private:
    float distThreshold,
          errorThreshold;
public:
    typedef pair<unsigned,unsigned> PUU;

    VMHeuristicByAngVariation();

    // VertexMatcher interface
public:
    bool load(ConfigLoader &config);
    float vertexMatch(Gaussian &gu, Gaussian &gv,
                      vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                      unsigned *matchEdges);

    float vertexMatch_orig(Gaussian &gu, Gaussian &gv,
                      vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                      unsigned *matchEdges);

    float vertexMatch(Gaussian &gu, Gaussian &gv,
                      vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                      vector<MatchInfoWeighted> &bestMatch);

    float vertexMatch_orig(Gaussian &gu, Gaussian &gv,
                      vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                      vector<MatchInfoWeighted> &matchEdges);

private:

    float findCompativelEdgesByLenght(unsigned sourceEdgeId,
                                     vector<GraphLink *> &u, vector<GraphLink *> &v,
                                     vector<PUU> &match);

    float findCompativelEdges(unsigned uBeginEdgeId, unsigned vBeginEdgeId,
                                     vector<GraphLink *> &u, vector<GraphLink *> &v,
                                     vector<MatchInfoWeighted> &match);

    float computeError();

    bool exploreBestEdgeMatch(float &baseError, float uAngDiff, float uEndLength,
                              unsigned vBeginId, unsigned &vEndId,int &vCount,
                              vector<GraphLink*> &v);

    float computeEdgeAngDiff(GraphLink *uBegin, GraphLink *uEnd);

    float computeScoreBetweenEdges(GraphLink *uBegin, GraphLink *uEnd,
                                   GraphLink *vBegin, GraphLink *vEnd);

    void findGraphMatchWithInitialGuess(vector<vector<GraphLink *> > &u, vector<vector<GraphLink *> > &v,
                                        MatchInfoExtended &initialVertexMatch,
                                        vector<MatchInfoExtended> &graphMatch);

    void showInitialGuess();

};

#endif // VMHEURISTICBYANGVARIATION_H
