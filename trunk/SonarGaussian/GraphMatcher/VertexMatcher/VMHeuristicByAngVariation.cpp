#include "VMHeuristicByAngVariation.h"
#include "GraphMatcher/GraphMatcher.h"
#include "Drawing/Drawing.h"

#include <queue>

VMHeuristicByAngVariation::VMHeuristicByAngVariation()
{
}


bool VMHeuristicByAngVariation::load(ConfigLoader &config)
{
    bool gotSomeConfig=false;
    float fv;

    if(config.getFloat("VMHeuristicByAngVariation","distThreshold",&fv))
    {
        distThreshold= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMHeuristicByAngVariation","errorThreshold",&fv))
    {
        errorThreshold= fv;
        gotSomeConfig=true;
    }

    return gotSomeConfig;
}

/**
 * @brief
 *
 * @param gu
 * @param gv
 * @param eu
 * @param ev
 * @param matchEdges
 * @return float
 */
float VMHeuristicByAngVariation::vertexMatch(Gaussian &gu, Gaussian &gv,
                                           vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                                           unsigned *matchEdges)
{
    unsigned bestMatch=0;

    // This method need more than one edge to match
    if(eu.size() <= 1 || ev.size() <= 1)
        return 1.f;

    // Reference edge guess
    vector<MatchInfoWeighted> match;

    float bestSolutionError = errorThreshold;

    // Find good reference edges guess by length compatibility
    for(unsigned i = 0 ; i < eu.size() ; i++)
    {
        for(unsigned j = 0 ; j < ev.size() ; j++)
        {
            float distError = fabs(eu[i]->p - ev[j]->p);
            if(distError < distThreshold)
            {
                float soluctionError= findCompativelEdges(i,j,
                                    eu,ev,match);

                if(match.size() > bestMatch)
                {
                    bestMatch = match.size();
                    bestSolutionError = soluctionError;
                }else if(match.size() == bestMatch && bestSolutionError < soluctionError)
                {
                    bestMatch = match.size();
                    bestSolutionError = soluctionError;
                }
            }
        }
    }
    if(matchEdges != 0x0)
        *matchEdges = bestMatch;

    return bestSolutionError/errorThreshold;
}

float VMHeuristicByAngVariation::vertexMatch_orig(Gaussian &gu, Gaussian &gv, vector<GraphLink *> &eu, vector<GraphLink *> &ev, unsigned *matchEdges)
{
    vector<vector<PUU> > match(1);
    unsigned nSingleSolutions = 0, nSolutions=0;

    float meanError;

    for(unsigned i = 0 ; i < eu.size() ; i++)
    {
        meanError = findCompativelEdgesByLenght(i,eu,ev,match[nSingleSolutions]);

        for(unsigned j = 0 ; j < match[nSingleSolutions].size();j++)
            cout << match[nSingleSolutions][j].first << " -> " << match[nSingleSolutions][j].second << endl;
        cout << i << " - mean Error "<< meanError << endl;

        if(meanError < distThreshold)
        {
            nSolutions+=match[nSingleSolutions].size();
            nSingleSolutions++;
            match.push_back(vector<PUU>() );
        }
    }

    vector<MatchInfoWeighted> solutions[nSolutions];
    unsigned bestSoluction, bestSize=0, soluctionId=0;
    float bestSoluctionError=99999.9f;

    for(unsigned j = 0 ; j < nSingleSolutions ; j++)
    {
        for(unsigned i = 0 ; i < match[j].size() ; i++)
        {
            float soluctionError= findCompativelEdges(match[j][i].first,
                                                      match[j][i].second,
                                eu,ev,solutions[soluctionId]);


            cout << "Soluction "
                 << match[j][i].first
                 << " -> "
                 << match[j][i].second
                 << " : er "
                 << soluctionError
                 << " : size "
                 << solutions[soluctionId].size()
                 << endl;

            if(solutions[soluctionId].size()> 0 )
            {

    //            if(soluctionError < bestSoluctionError)
                // I think size is more important than error
                if(solutions[soluctionId].size() > bestSize ||
                   (solutions[soluctionId].size() == bestSize &&
                    soluctionError < bestSoluctionError)  )
                {

                    bestSoluction = soluctionId;
                    bestSize = solutions[soluctionId].size();
                    bestSoluctionError = soluctionError;
                }
            }
            soluctionId++;
        }
    }

    // If a soluction was found
    if(bestSize>0)
    {
        if(matchEdges != 0x0)
            *matchEdges = solutions[bestSoluction].size();

        cout << "Best soluction was "
             << "error "
             << bestSoluctionError
             << " size "
             << solutions[bestSoluction].size()
             << endl;
    }
    cout << "No soluction found!" << endl;

    return bestSoluctionError;
}


/**
 * @brief
 *  We explore first more matchs possible and second
 * matchs with less error. Always respecting errorThreshold
 * and distThreshold.
 * @param u
 * @param v
 * @param matchEdges
 * @return float
 */
float VMHeuristicByAngVariation::vertexMatch(Gaussian &gu, Gaussian &gv,
                                           vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                                           vector<MatchInfoWeighted> &bestMatch)
{
    bestMatch.clear();

    // This method need more than one edge to match
    if(eu.size() <= 1 || ev.size() <= 1)
        return 1.f;

    // Reference edge guess
    vector<MatchInfoWeighted> match;

    float bestSolutionError = errorThreshold;

    // Find good reference edges guess by length compatibility
    for(unsigned i = 0 ; i < eu.size() ; i++)
    {
        for(unsigned j = 0 ; j < ev.size() ; j++)
        {
            float distError = fabs(eu[i]->p - ev[j]->p);
            if(distError < distThreshold)
            {
                float soluctionError= findCompativelEdges(i,j,
                                    eu,ev,match);

                if(match.size() > bestMatch.size())
                {
                    bestMatch = match;
                    bestSolutionError = soluctionError;
                }else if(match.size() == bestMatch.size() && bestSolutionError < soluctionError)
                {
                    bestMatch = match;
                    bestSolutionError = soluctionError;
                }
            }
        }
    }

    return bestSolutionError/errorThreshold;
}

float VMHeuristicByAngVariation::vertexMatch_orig(Gaussian &gu, Gaussian &gv,
                                              vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                                              vector<MatchInfoWeighted> &matchEdges)
{
    // Reference edge guess
    vector<vector<PUU> > match(1);
    unsigned nSingleSolutions = 0, nSolutions=0;

    float meanError;

    // Find good reference edges guess by length compatibility
    for(unsigned i = 0 ; i < eu.size() ; i++)
    {
        meanError = findCompativelEdgesByLenght(i,eu,ev,match[nSingleSolutions]);

        for(unsigned j = 0 ; j < match[nSingleSolutions].size();j++)
            cout << match[nSingleSolutions][j].first << " -> " << match[nSingleSolutions][j].second << endl;
        cout << i << " - mean Error "<< meanError << endl;

        if(meanError < distThreshold)
        {
            nSolutions+=match[nSingleSolutions].size();
            nSingleSolutions++;
            match.push_back(vector<PUU>() );
        }
    }

    vector<MatchInfoWeighted> solutions[nSolutions];
    unsigned bestSoluction, bestSize=0, soluctionId=0;
    float bestSoluctionError=99999.9f;

    for(unsigned j = 0 ; j < nSingleSolutions ; j++)
    {
        for(unsigned i = 0 ; i < match[j].size() ; i++)
        {
            float soluctionError= findCompativelEdges(match[j][i].first,
                                                      match[j][i].second,
                                eu,ev,solutions[soluctionId]);


            cout << "Soluction "
                 << match[j][i].first
                 << " -> "
                 << match[j][i].second
                 << " : er "
                 << soluctionError
                 << " : size "
                 << solutions[soluctionId].size()
                 << endl;

            if(solutions[soluctionId].size()> 0 )
            {

    //            if(soluctionError < bestSoluctionError)
                // I think size is more important than error
                if(solutions[soluctionId].size() > bestSize ||
                   (solutions[soluctionId].size() == bestSize &&
                    soluctionError < bestSoluctionError)  )
                {

                    bestSoluction = soluctionId;
                    bestSize = solutions[soluctionId].size();
                    bestSoluctionError = soluctionError;
                }
            }
            soluctionId++;
        }
    }

    // If a soluction was found
    if(bestSize>0)
    {
        matchEdges = solutions[bestSoluction];
        cout << "Best soluction was "
             << "error "
             << bestSoluctionError
             << " size "
             << solutions[bestSoluction].size()
             << endl;
    }
    cout << "No soluction found!" << endl;

    return bestSoluctionError;
}


/**
 * @brief
 *  Find possible edges match looking its
 * length. This method can be used to estimate
 * the reference edges before use findCompativelEdges.
 * @param sourceEdgeId - Reference edge of vertex u.
 * @param u - Vertex u
 * @param v - Vertex v that we will search best match.
 * @param match - The matchs found ( sourceEdgeId ,
 *  v edge id found )
 * @return float - Mean scalene error.
 */
float VMHeuristicByAngVariation::findCompativelEdgesByLenght(unsigned sourceEdgeId,
                                                             vector<GraphLink *> &u, vector<GraphLink *> &v,
                                                             vector<PUU> &match)
{
    GraphLink *srcEdge = u[sourceEdgeId];
    float accDistDiff=0;

    for(unsigned i = 0 ; i < v.size() ; i++)
    {
        GraphLink *targetEdge = v[i];
        float distDiff = fabs(targetEdge->p - srcEdge->p);
        if(  distDiff <= distThreshold)
        {
            accDistDiff+=distDiff;
            match.push_back(PUU(sourceEdgeId,i));
        }
    }
    if(match.size()>0)
        return accDistDiff/match.size();
    return 999999.99f;
}

/**
 * @brief
 *  Find the best edges match between two vertices with
 * a reference edge.
 * @param uBeginEdgeId - Reference edge ID from vertex u
 * @param vBeginEdgeId - Reference edge ID from vertex v
 * @param u - Vertex u
 * @param v - Vertex v
 * @param match - Matchs found ( ( u edge ID, v edge ID ) ,
 * (scalene error , 0.f) )
 * @return float - Mean error of match.
 */
float VMHeuristicByAngVariation::findCompativelEdges(unsigned uBeginEdgeId, unsigned vBeginEdgeId,
                                                     vector<GraphLink *> &u, vector<GraphLink *> &v,
                                                     vector<MatchInfoWeighted> &match)
{
    match.clear();

    // This method need more than one edge to match
    if(u.size() <= 1 || v.size() <= 1)
        return 999999.9f;

    unsigned uEndEdgeId = (uBeginEdgeId+1)%u.size(),
             vEndEdgeId = (vBeginEdgeId+1)%v.size();
            // Less our two edges that we will analyze (begin and end)

    int uCount =u.size()-2, vCount =v.size()-2; // Remained edges

    float errorAcc= fabs(u[uBeginEdgeId]->p - v[vBeginEdgeId]->p);

    // Add reference edges to solution
    match.push_back(MatchInfoWeighted(uBeginEdgeId,vBeginEdgeId,errorAcc));

    // While u and v have at least 2 adj edges to match
    while(uCount >= 0 && vCount >= 0)
    {
        // Compute error between edges
        float uAngDiff = computeEdgeAngDiff(u[uBeginEdgeId],u[uEndEdgeId]),
              vAngDiff = computeEdgeAngDiff(v[vBeginEdgeId],v[vEndEdgeId]),
              angDiff = std::fabs(uAngDiff-vAngDiff),
              error =  GraphMatcher::scaleneError(angDiff,u[uEndEdgeId]->p,v[vEndEdgeId]->p);

        if(error < errorThreshold)
        {
            // Match was found !
            match.push_back(MatchInfoWeighted(uEndEdgeId,vEndEdgeId,error));
            errorAcc+= error;

            uBeginEdgeId = uEndEdgeId;
            vBeginEdgeId = vEndEdgeId;

            uEndEdgeId = (uEndEdgeId+1)%u.size(),
            vEndEdgeId = (vEndEdgeId+1)%v.size(),
             // Less one edge for each vertex who now are used in uEndId and vEndId
             uCount--; vCount--;
        }else
        {
            if(uAngDiff < vAngDiff)
            {
                uEndEdgeId = (uEndEdgeId+1)%u.size();
                uCount--;
            }else
            {
                vEndEdgeId = (vEndEdgeId+1)%v.size();
                vCount--;
            }
        }
    }

    // No match found
    if(match.size() == 1) // First match
    {
        // Base edge (uBegin,vBegin) not confirmed!
//        cout << "Base edge not confirmed!" << endl;
        match.clear();
        return 999999.9f;
    }
    return errorAcc/(match.size());
}

/**
 * @brief
 *
 *  u and v are vertices of two distict graphs
 *
 * @param baseError - Inicial error between two initial edges from u and
 * two initial edges from v. It's will be changed to the best error found.
 * @param uAngDiff - Ang diff between two initial edges from u.
 * @param uEndLength - Length of last initial edge from u.
 * @param vBeginId - Id of first initial edge from v.
 * @param vEndId - Id of last initial edge from v, it's will be changed
 * to Id of best last edge found.
 * @param vCount - Number of remainder edges of v, it's will be updated
 * decresing edges used by this search.
 * @param v - A vertex that will be explored.
 * @return bool - true if an aceptable match was found.
 */
bool VMHeuristicByAngVariation::exploreBestEdgeMatch(float &baseError, float uAngDiff, float uEndLength, unsigned vBeginId, unsigned &vEndId, int &vCount, vector<GraphLink *> &v)
{
    // Compare current error with next v edge error
    unsigned vNextId = vEndId;

    int      vSearchCount = vCount;

    float vAngDiff = 99999.9f,
          nextError = 99999.9f,
          angDiff;

    bool decreasingErro= false,
         aceptableMatch=false;

    do
    {
        vNextId = (vNextId+1)%v.size(); // Go next adj edge of v
        vSearchCount--; // Decrease one edge

        // Comupte next error
        vAngDiff = computeEdgeAngDiff(v[vBeginId],v[vNextId]);
        angDiff = std::fabs(uAngDiff - vAngDiff);
//        nextError = GraphMatcher::scaleneError(angDiff,uEndLength,v[vNextId]->p);

        // Checks whether it is worth continuing searching
        decreasingErro = nextError < baseError;

        // If a new match is aceptable
        if(nextError <= errorThreshold)
        {
            // Update base error
            baseError = nextError;
            // Update last edge id from v
            vEndId = vNextId;
            // Update remainder edges from v
            vCount = vSearchCount;

            aceptableMatch=true;
        }

    }while(vSearchCount > 0 && decreasingErro && !aceptableMatch);

    return aceptableMatch;
}

float VMHeuristicByAngVariation::computeEdgeAngDiff(GraphLink *uBegin, GraphLink *uEnd)
{
    if(uBegin->ang <= uEnd->ang)
    {
        return uEnd->ang - uBegin->ang;
    }else
    {
        return 360.f - uBegin->ang + uEnd->ang;
    }
//    float angDiff = fabs(uEnd->ang - uBegin->ang);
//    if(angDiff>180.f) angDiff = 360.f - angDiff;
//    return angDiff;
}

float VMHeuristicByAngVariation::computeScoreBetweenEdges(GraphLink *uBegin, GraphLink *uEnd, GraphLink *vBegin, GraphLink *vEnd)
{
    float uAngDiff = fabs(uEnd->ang - uBegin->ang),
          vAngDiff = fabs(vEnd->ang - vBegin->ang);

    if(uAngDiff>180.f) uAngDiff = 360.f - uAngDiff;
    if(vAngDiff>180.f) vAngDiff = 360.f - vAngDiff;

    float angDiff = fabs(uAngDiff-vAngDiff);

    return GraphMatcher::scaleneError(angDiff,uEnd->p,vEnd->p);
}

void VMHeuristicByAngVariation::findGraphMatchWithInitialGuess(vector<vector<GraphLink *> > &u, vector<vector<GraphLink *> > &v,
                                                             MatchInfoExtended &initialVertexMatch,
                                                             vector<MatchInfoExtended> &graphMatch)
{
    typedef pair<PUU,PUU> PUUPUU;
    queue<PUUPUU> q; // vertex id and reference edge id of graph u and graph v

    // Please use char because vector optimize memory usage
    // and decrease speed performance with bool
    vector<char> uV(u.size(),false), // visited value of graph u
                 vV(v.size(),false); // visited value of graph u

    uV[initialVertexMatch.uID] = true;
    vV[initialVertexMatch.vID] = true;


//    while(!q.empty())
//    {
//        vector<PUUPUU> &initEdgeMatch = initialVertexMatch.edgeMatchInfo;
//        for(unsigned i = 0 ; i < initEdgeMatch.size(); i++)
//        {
//            q.push(MatchInfoWeighted(
//                       PUU(initialVertexMatch.uID, initEdgeMatch[i].uID),
//                       PUU(initialVertexMatch.vID, initEdgeMatch[i].vID)
//                          ));
//        }
    //    }
}

void VMHeuristicByAngVariation::showInitialGuess()
{

//    Drawing::drawVertexMatch(imgVertexMatch,Size2i(1000,600),
//                             *sds[leftId],*sds[rightId],
//                             leftSelecGaussian,rightSelecGaussian,
//                             initialVertexMatch.edgeMatchInfo,
//                             false,true,
//                             Scalar(0,0,255),Scalar(0,255,0),2);

//    imshow("VertexMatch",imgVertexMatch);

}
