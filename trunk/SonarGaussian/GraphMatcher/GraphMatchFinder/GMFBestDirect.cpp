#include "GMFBestDirect.h"

bool GMFBestDirect::findBest(unsigned uId, unsigned &vId)
{
    vector<GraphLink*> &u1 = current_sd1->graph[uId];
    vector<vector<GraphLink*> > &gv = current_sd2->graph;

    unsigned bestMatch=0,edgeMatch=0;

    int matchId = -1;

    float error, bestError=99999.9f;

    if(u1.size() < minEdgeToMatch)
        return false;

    // For each gaussian v
    for(unsigned v = 0 ; v < gv.size(); v++)
    {
        if(gv[v].size() < minEdgeToMatch) continue;

        error = m_vertexMatcher->vertexMatch(current_sd1->gaussians[uId],
                                             current_sd2->gaussians[v],
                                             u1,gv[v],&edgeMatch);

        if(edgeMatch < minEdgeToMatch) continue;

        if(edgeMatch > bestMatch ||
     (edgeMatch == bestMatch  &&  error < bestError))
        {
            bestMatch = edgeMatch;
            bestError = error;
            matchId = v;
        }
    }

    if(matchId < 0)
        return false;

    vId = matchId;
    return true;
}

void GMFBestDirect::fastMatchCheck(vector<MatchInfo> &vertexMatch)
{
    vector<MatchInfo> vertexMatchCopy(vertexMatch);
    vector<Gaussian> &g1 = current_sd1->gaussians,
                     &g2 = current_sd2->gaussians;
    unsigned nMatchs = vertexMatch.size();

    vertexMatch.clear();
    if(nMatchs > 1)
    for(unsigned i = 0, j=1; j != 0; j= (j+1)%nMatchs)
    {
        MatchInfo &current = vertexMatchCopy[i],
                  &next = vertexMatchCopy[j];

        if( fabs(
            dist(g1[current.uID], g1[next.uID])
                            -
            dist(g2[current.vID], g2[next.vID]))
                         <=   25.f
           )
        {
            vertexMatch.push_back(next);
            i++;
        }
    }
}

void GMFBestDirect::fastMatchCheck2(vector<MatchInfo> &vertexMatch)
{
    vector<MatchInfo> vertexMatchCopy(vertexMatch);
    vector<Gaussian> &g1 = current_sd1->gaussians,
                     &g2 = current_sd2->gaussians;

    unsigned nMatchs = vertexMatch.size();

    vertexMatch.clear();

    if(nMatchs <= 1)
        return ;

    Gaussian &ref1 = g1[vertexMatchCopy[0].uID],
             &ref2 = g2[vertexMatchCopy[0].vID];

    vertexMatch.push_back(vertexMatchCopy[0]);

    for(unsigned i = 1; i < nMatchs; i++)
    {
        MatchInfo &current = vertexMatchCopy[i];

        if( fabs(
              dist( ref1 , g1[current.uID ] )
                          -
              dist( ref2 , g2[current.vID ] ) )
                         <= 25.f
          )
        {
            vertexMatch.push_back(current);
        }
    }

    if(vertexMatch.size() == 1)
        vertexMatch.clear();
}

GMFBestDirect::GMFBestDirect():
    minEdgeToMatch(8)
{
}

bool GMFBestDirect::load(ConfigLoader &config)
{
    return true;

}

void GMFBestDirect::findMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                              vector<MatchInfo> &vertexMatch)
{
    current_sd1 = sd1;
    current_sd2 = sd2;

    vector<vector<GraphLink *> > &g1 = sd1->graph;

    unsigned v;

    for(unsigned u = 0; u < g1.size() ; u++)
    {
        if(findBest(u,v))
        {            
            vertexMatch.push_back(MatchInfo(u,v));
        }
    }

    fastMatchCheck(vertexMatch);
    fastMatchCheck2(vertexMatch);

    if(vertexMatch.size() <= 2)
        vertexMatch.clear();

}

void GMFBestDirect::findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2, vector<MatchInfoExtended> &matchInfo)
{

}
