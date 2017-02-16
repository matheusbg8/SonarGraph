#include "VMbyEdgeLength.h"

VMbyEdgeLength::VMbyEdgeLength()
{
}

bool VMbyEdgeLength::load(ConfigLoader &config)
{

}

float VMbyEdgeLength::vertexMatch(Gaussian &gu, Gaussian &gv,
                                  vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                                  unsigned *matchEdges)
{

}

float VMbyEdgeLength::vertexMatch(Gaussian &gu, Gaussian &gv,
                                  vector<GraphLink *> &u, vector<GraphLink *> &v,
                                  vector<MatchInfoWeighted> &matchEdges)
{

    vector<MatchInfoWeighted> matchs;
    ha.clear();
    ha.hungarian(matchs);
}
