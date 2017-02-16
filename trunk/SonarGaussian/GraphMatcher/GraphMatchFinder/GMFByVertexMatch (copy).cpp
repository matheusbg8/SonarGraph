#include "GMFByVertexMatch.h"

// It's a temp. struct to store matchs
class Match
{
public:
    unsigned bestMatch, secBestMatch, destId;
    Match(unsigned bestMatch, unsigned secBestMatch, unsigned destId):
        bestMatch(bestMatch), secBestMatch(secBestMatch), destId(destId)
    { }
};

void GMFByVertexMatch::fastMatchCheck(SonarDescritor *sd1, SonarDescritor *sd2,
                                     vector<MatchInfo> &vertexMatch, unsigned basedMatchId,
                                     vector<MatchInfo> &result)
{
    vector<Gaussian> &g1 = sd1->gaussians,
                     &g2 = sd2->gaussians;
    unsigned nMatchs = vertexMatch.size();

    result.clear();
    if(nMatchs < 3)
        return;

    MatchInfo &refMatch = vertexMatch[basedMatchId];

    for(unsigned refMatchId = basedMatchId,
                 nexMatchId=(refMatchId+1)%nMatchs;

                nexMatchId != basedMatchId;

         nexMatchId= (nexMatchId+1)%nMatchs)
    {
        MatchInfo &current = vertexMatch[refMatchId],
                  &next = vertexMatch[nexMatchId];

        if( fabs(
            dist(g1[current.uID], g1[next.uID])
                            -
            dist(g2[current.vID], g2[next.vID]))
                         <=   25.f
           &&
            fabs(
            dist(g1[refMatch.uID], g1[next.uID])
                            -
            dist(g2[refMatch.vID], g2[next.vID]))
                         <=   25.f
           )
        {
            result.push_back(next);
            refMatchId= (refMatchId+1)%nMatchs;
        }
    }
    if(result.size() < 3)
        result.clear();
}

void GMFByVertexMatch::fastMatchCheck2(SonarDescritor *sd1, SonarDescritor *sd2,
                                       vector<MatchInfo> &vertexMatch, unsigned basedMatchId,
                                       vector<MatchInfo> &result)
{
    vector<Gaussian> &g1 = sd1->gaussians,
                     &g2 = sd2->gaussians;

    unsigned nMatchs = vertexMatch.size();

    result.clear();
    if(nMatchs <= 1)
        return ;

    Gaussian &ref1 = g1[vertexMatch[basedMatchId].uID],
             &ref2 = g2[vertexMatch[basedMatchId].vID];

    result.push_back(vertexMatch[basedMatchId]);

    for(unsigned i = (basedMatchId+1)%nMatchs;
        i != basedMatchId; i= (i+1)%nMatchs)
    {
        MatchInfo &current = vertexMatch[i];

        if( fabs(
              dist( ref1 , g1[current.uID ] )
                          -
              dist( ref2 , g2[current.vID ] ) )
                         <= 25.f
          )
        {
            result.push_back(current);
        }
    }
    if(result.size() == 1)
        result.clear();
}

GMFByVertexMatch::GMFByVertexMatch():
    minEdgeToMatch(4),
    minEdgeDiffToAcceptMatch(1)
{

}

bool GMFByVertexMatch::load(ConfigLoader &config)
{
    return true;
}

void GMFByVertexMatch::findMatch(SonarDescritor *sd1, SonarDescritor *sd2, vector<MatchInfo> &vertexMatch)
{
    vector< vector<GraphLink*> > &g1 = sd1->graph,
                                 &g2 = sd2->graph;

    vector <Match> matchFromVtoU(g2.size(),Match(0,0,0));

    unsigned bestMatch,secBestMatch,
             edgeMatch,
             matchId;

    float error;

    // For each gaussian u
    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1[u].size() < minEdgeToMatch) continue;

        bestMatch = secBestMatch = 0;

        // For each gaussian v
        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2[v].size() < minEdgeToMatch) continue;

            error = m_vertexMatcher->vertexMatch(sd1->gaussians[u],sd2->gaussians[v],
                                                 g1[u],g2[v],&edgeMatch);

            if(edgeMatch < minEdgeToMatch) continue;

            if(edgeMatch > bestMatch)
            {
                secBestMatch = bestMatch;
                bestMatch = edgeMatch;
                matchId = v;
            }else if(edgeMatch > secBestMatch)
            {
                secBestMatch = edgeMatch;
            }
        }


        // If it was a match from U to V (g1 to g2)
        if(bestMatch > 0)
        {
            // If there is two very similar matchs
            if(bestMatch - secBestMatch  < minEdgeDiffToAcceptMatch)
            { // Don't take the match
                continue;
            }

            // Save this match as match from g2 to g1 (inverse match)
            Match &match = matchFromVtoU[matchId];

            if(bestMatch > match.bestMatch)
            {
                match.secBestMatch = match.bestMatch;
                match.bestMatch = bestMatch;
                match.destId = u;
            }else if(bestMatch > match.secBestMatch)
            {
                match.secBestMatch = bestMatch;
            }
        }
    }

    for(unsigned v = 0 ; v < g2.size() ; v++)
    {
        Match &match = matchFromVtoU[v];

        // If it was a match from V to U (g2 to g1)
        if(match.bestMatch > 0)
        {
            // If there isn't two very similar matchs
            if(match.bestMatch - match.secBestMatch >= minEdgeDiffToAcceptMatch)
            { // Take this match
                vertexMatch.push_back(MatchInfo(match.destId, v));
            }
        }
    }
//    fastMatchCheck(sd1,sd2, vertexMatch);
    vector<MatchInfo> bestFilterResult;
    for(unsigned i = 0 ; i < vertexMatch.size(); i++)
    {
        vector<MatchInfo> filterResult;
        fastMatchCheck(sd1,sd2,vertexMatch,i,filterResult);
        if(filterResult.size() > bestFilterResult.size())
        {
            bestFilterResult = filterResult;
        }
    }
    vertexMatch = bestFilterResult;
}

void GMFByVertexMatch::findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2, vector<MatchInfoExtended> &matchInfo)
{

}
