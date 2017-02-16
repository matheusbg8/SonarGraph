#include "GMFByEdgeExploration.h"
#include <queue>

bool GMFByEdgeExploration::findBest(unsigned uId, unsigned &vId)
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

bool GMFByEdgeExploration::explore(unsigned uId, unsigned vId,
                                   vector<MatchInfoWeighted> &vertexMatch)
{
    vector<MatchInfoWeighted> edgeMatch;

    vector<vector<GraphLink*> > &g1 =current_sd1->graph,
                                &g2 =current_sd2->graph;

    queue<pair<unsigned, unsigned> > q;
    q.push(make_pair(uId, vId));

    unsigned next_uId, next_vId;

    while(!q.empty())
    {
        uId = q.front().first;
        vId = q.front().second;
        q.pop();

        if(lb1[uId] == -1 && lb2[vId] == -1)
        {
            m_vertexMatcher->vertexMatch(current_sd1->gaussians[uId],
                                         current_sd2->gaussians[vId],
                                         g1[uId],g2[vId],
                                         edgeMatch);

            for(unsigned i = 0; i < edgeMatch.size(); i++)
            {
                next_uId = g1[uId][edgeMatch[i].uID]->dest;
                next_vId = g2[vId][edgeMatch[i].vID]->dest;
                int lb1v = lb1[next_uId], lb2v = lb2[next_vId];

                if(lb1v ==-1 && lb2v ==-1)
                {   // They do not have label yet
                    q.push(make_pair(next_uId,next_vId));
                }else if(lb1v >= 0 && lb2v >=0)
                {   // They have labels
                    if(lb1v != lb2v)
                    { // Wrong label
//                        return false;
                        lb1[next_uId] = -2;
                        lb2[next_vId] = -2;
                    }else
                    { // Cicle detected
                        vertexMatch.push_back(MatchInfoWeighted(uId,vId));
                        lb1[next_uId] = -2;
                        lb2[next_vId] = -2;
                    }
                }
            }

            //Create labels
            lb1[uId] = lb2[vId] = lbCount;
            lbCount++;
        }
    }
    return true;
}

bool GMFByEdgeExploration::fastMatchCheck(vector<MatchInfoWeighted> &vertexMatch)
{
    vector<Gaussian> &g1 = current_sd1->gaussians,
                     &g2 = current_sd2->gaussians;
    unsigned nMatchs = vertexMatch.size();

    if(nMatchs <= 1)
        return false;

    for(unsigned i = 0, j=1; j != 0; j= (j+1)%nMatchs)
    {
        MatchInfoWeighted &current = vertexMatch[i],
                          &next = vertexMatch[j];

        if( fabs(
            dist(g1[current.uID], g1[next.uID])
                            -
            dist(g2[current.vID], g2[next.vID]))
                         <=   25.f
           )
        {
            i++;
        }else
        {
            return false;
        }
    }
    return true;
}

bool GMFByEdgeExploration::fastMatchCheck2(vector<MatchInfoWeighted> &vertexMatch)
{
    vector<Gaussian> &g1 = current_sd1->gaussians,
                     &g2 = current_sd2->gaussians;

    unsigned nMatchs = vertexMatch.size();
    if(nMatchs <= 1)
        return false;

    Gaussian &ref1 = g1[vertexMatch[0].uID],
             &ref2 = g2[vertexMatch[0].vID];

    for(unsigned i = 1; i < nMatchs; i++)
    {
        MatchInfoWeighted &current = vertexMatch[i];

        if( fabs(
              dist( ref1 , g1[current.uID ] )
                          -
              dist( ref2 , g2[current.vID ] ) )
                         > 25.f
          )
        {
            return false;
        }
    }
    return true;
}

GMFByEdgeExploration::GMFByEdgeExploration():
    minEdgeToMatch(4),
    minEdgeDiffToAcceptMatch(1)
{
}

bool GMFByEdgeExploration::load(ConfigLoader &config)
{

}

void GMFByEdgeExploration::findMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                                     vector<MatchInfo> &vertexMatch)
{
    current_sd1 = sd1;
    current_sd2 = sd2;

    vector<vector<GraphLink *> > &g1 = sd1->graph,
                                 &g2 = sd2->graph;

    vector<MatchInfoWeighted> tmpVertexMatch;

    unsigned v;

    for(unsigned u = 0; u < g1.size() ; u++)
    {
        lb1.clear();
        lb2.clear();

        lb1.resize(g1.size(),-1);
        lb2.resize(g2.size(),-1);

        lbCount = 0;
        if(findBest(u,v))
        {
            cout << "best " << u << " , " << v << endl;
            if(explore(u,v,tmpVertexMatch))
            {
                if(fastMatchCheck(tmpVertexMatch) &&
                   fastMatchCheck2(tmpVertexMatch))
                {
                    break;
                }
            }
            tmpVertexMatch.clear();
        }
    }

    /*
    Inicia 1 pra um
    Explora todos vizinhos atribuindo label 0-n
    Se ja tem label compara
        Se for diferente sub 1
        Se for igual soma 1
    Se n tem explora
    */

    vertexMatch.clear();
    vertexMatch.reserve(tmpVertexMatch.size());

    for(unsigned i = 0 ; i < tmpVertexMatch.size(); i++)
    {
        MatchInfoWeighted &m = tmpVertexMatch[i];
        if(m.score > -2.f)
        {
            vertexMatch.push_back(MatchInfo(m.uID,m.vID));
        }
    }

}

void GMFByEdgeExploration::findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                                          vector<MatchInfoExtended> &matchInfo)
{

}
