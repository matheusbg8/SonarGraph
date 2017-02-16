#include "VMScalenePC.h"

#include <algorithm>
#include "GraphMatcher/GraphMatcher.h"


VMScalenePC::VMScalenePC():
    epsMax(30.f),
    gaussianRatioW(1.f)
{
}

bool VMScalenePC::load(ConfigLoader &config)
{
    bool gotSomeConfig=false;
    float fv;

    if(config.getFloat("VMScalenePC","epsMax",&fv))
    {
        epsMax= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMScalenePC","gaussianRatioW",&fv))
    {
        gaussianRatioW= fv;
        gotSomeConfig=true;
    }

    return gotSomeConfig;
}

float VMScalenePC::vertexMatch(Gaussian &gu, Gaussian &gv,
                             vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                             unsigned *matchEdges)
{
    // ** We consider that edges are in an increasing order
    std::sort(eu.begin(),eu.end(),compRelativeAng);
    std::sort(ev.begin(),ev.end(),compRelativeAng);

    unsigned i=0, j=0, edgeMatchCount=0;

    float edgeErrorScore=0.f;

    // For each edge i in vertex u and each edge j in vertex v
    while( i < eu.size() && j < ev.size())
    {
        // uP and vP are lenght of each edge
        float uP = eu[i]->p, vP = ev[j]->p,
              diffAng = fabs(eu[i]->rAng - ev[j]->rAng),
              erroP = sqrt(uP*uP + vP*vP -2.f*uP*vP*cos( (diffAng*M_PI)/180.f ));
        // erroP is a scalene error between the edges

        if(erroP < epsMax)
        {
            edgeErrorScore+= erroP;
            edgeMatchCount++;
            i++; j++;
        }else
        {
            if(eu[i]->ang < ev[j]->ang)
                i++;
            else j++;
        }
    }
    if(edgeMatchCount > 0)
    {
        edgeErrorScore/= edgeMatchCount;
    }

    if(matchEdges != 0x0)
        *matchEdges = edgeMatchCount;

    return edgeErrorScore +
       std::fabs(gu.x/gu.y - gv.x/gv.y)*gaussianRatioW;
}


/**
 * @brief
 *
 * @param u
 * @param v
 * @param matchEdges
 * @return float
 */
float VMScalenePC::vertexMatch(Gaussian &gu, Gaussian &gv,
                             vector<GraphLink *> &u, vector<GraphLink *> &v,
                             vector<MatchInfoWeighted> &matchEdges)
{
    // ** We consider that edges are in an increasing order
    std::sort(u.begin(),u.end(),compRelativeAng);
    std::sort(v.begin(),v.end(),compRelativeAng);

    unsigned i=0, j=0, edgeMatch=0;

    float errorScore=0.f;

    // For each edge i in vertex u and each edge j in vertex v
    while( i < u.size() && j < v.size())
    {
        // uP and vP are the lenghts of edges
        float uP = u[i]->p, vP = v[j]->p,
              diffAng = fabs(u[i]->rAng - v[j]->rAng), // Ang diff between edges u and v
              erroP = GraphMatcher::scaleneError(diffAng,uP,vP);

        // erroP is a scalene error between the edges

        if(erroP < epsMax)
        {
            errorScore+= erroP;
            matchEdges.push_back(MatchInfoWeighted(i,j,erroP));
            edgeMatch++;
            i++; j++;
        }else
        {
            if(u[i]->ang < v[j]->ang)
                i++;
            else j++;
        }
    }

    if(edgeMatch > 0)
    {
        errorScore/= edgeMatch;
    }

    return errorScore;

}
