#include "GMFHungarian.h"

GMFHungarian::GMFHungarian()
{
}

bool GMFHungarian::load(ConfigLoader &config)
{
    int iv;
    bool gotSomeConfig=false;

    if(config.getInt("GMFHungarian","minMatches",&iv))
    {
        minMatches = iv;
        gotSomeConfig = true;
    }
    return gotSomeConfig;
}

void GMFHungarian::findMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                             vector<MatchInfo> &vertexMatch)
{
    vector< vector<GraphLink*> > &g1 = sd1->graph,
                                 &g2 = sd2->graph;
    unsigned int edgeMatches=0u;
    float normError=0.f;

    unsigned int nV1 = g1.size(),
                 nV2 = g2.size();

    map1.setup(nV1);
    map2.setup(nV2);
    hu.clear();

    // For each gaussian u
    for(unsigned u = 0u ; u < nV1 ; u++)
    {
        if(g1[u].size() < minMatches) continue;

        for(unsigned v = 0u ; v < nV2; v++)
        {
            if(g2[v].size() < minMatches) continue;

            edgeMatches = 0u;
            normError = m_vertexMatcher->vertexMatch(sd1->gaussians[u],sd2->gaussians[v],
                                                 g1[u],g2[v],&edgeMatches);

            if(edgeMatches >= minMatches)
            {
                if( map1.count() < hu.maxVertex() && map2.count() < hu.maxVertex())
                {
                    hu.cost[map1.map(u)][map2.map(v)] = -(edgeMatches - normError);
                }else
                {
                    cout << "HUngarian:: Vertex limit achived!!" << endl;
                }
            }
        }
    }

    vector<MatchInfoWeighted> huMatch;

    hu.n = nV1;
    hu.max_match = nV2;

    hu.hungarian(huMatch);

    vertexMatch.resize(huMatch.size());

    for(unsigned i = 0; i < huMatch.size(); i++)
    {
        const MatchInfoWeighted &match = huMatch[i];

        vertexMatch[i] =
           MatchInfo(
                    map1.inv(match.uID),
                    map2.inv(match.vID));
    }

}

void GMFHungarian::findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                                  vector<MatchInfoExtended> &matchInfo)
{

}
