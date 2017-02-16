#include "VertexMatcher.h"

#ifndef VMWEGHTEDSUMPC_H
#define VMWEGHTEDSUMPC_H


/**
 * @brief This class compute vertex afinit using
 * vertex and edge informations. The affinit is
 * a weightedSum of errors. This approach needs
 * the weights parameters.
 *
 */
class VMWeghtedSumPC : public VertexMatcher
{
protected:
    // Edge Parameters
    float rhoMax,
          thetaMax,
          rhoW,
          thetaW,
          sigmaHitW,
          sigmaMissW,
    // Gaussin parameters
          angWeight,
          meanIWeight,
          stdXWeight,
          stdYWeight,
          stdIWeight;

public:
    VMWeghtedSumPC();


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

#endif // VMWEGHTEDSUMPC_H
