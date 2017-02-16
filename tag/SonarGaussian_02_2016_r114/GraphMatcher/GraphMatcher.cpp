#include "GraphMatcher.h"
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <stack>
#include <queue>

#include "Drawing/Drawing.h"

class VertexMatch
{
public:
    VertexMatch(float score=FLT_MAX, float angCorrection=0.f, int IDMathc=-1):
        score(score),angCorrection(angCorrection), IDMatch(IDMathc){}
    float score,angCorrection;
    int IDMatch;
};

bool compVertexMatch(const VertexMatch *a, const VertexMatch *b)
{
    return a->score < b->score;
}

void GraphMatcher::computeEdgeError(vector<GraphLink *> &u, vector<GraphLink *> &v,
                                    float *angScore, float *distScore,
                                    unsigned *edgeMatch, unsigned *edgeCut)
{
    // ** We consider that edges are in increasing order by ang

    sort(u.begin(),u.end(),compRelativeAng);
    sort(v.begin(),v.end(),compRelativeAng);

    unsigned i=0, j=0;
    (*edgeMatch)=0; (*edgeCut)=0;
    (*angScore)=0.f; (*distScore)=0.f;

    while( i < u.size() && j < v.size())
    {
        float diffDistance = fabs(u[i]->p - v[j]->p),
              diffAng = fabs(u[i]->rAng - v[j]->rAng);

        if(diffDistance < distThreshold && diffAng < angThreshold)
        {
            (*angScore)+= diffAng;
            (*distScore) += diffDistance;
            (*edgeMatch)++;
            i++; j++;
        }else
        {
            (*edgeCut)++;
            if(u[i]->ang < v[j]->ang)
                i++;
            else j++;
        }
    }
    if((*edgeMatch) > 0)
    {
        (*angScore)/= (*edgeMatch);
        (*distScore)/= (*edgeMatch);
    }
}

void GraphMatcher::computeEdgeErrorEscaleno(vector<GraphLink *> &u, vector<GraphLink *> &v, float *errorScore, unsigned *edgeMatch, unsigned *edgeCut)
{
    // ** We consider that edges are in an increasing order
    sort(u.begin(),u.end(),compRelativeAng);
    sort(v.begin(),v.end(),compRelativeAng);

    unsigned i=0, j=0;
    (*edgeMatch)=0; (*edgeCut)=0;
    (*errorScore)=0.f;

    while( i < u.size() && j < v.size())
    {
        float uP = u[i]->p, vP = v[j]->p,
              diffAng = fabs(u[i]->rAng - v[j]->rAng),
              erroP = sqrt(uP*uP + vP*vP -2.f*uP*vP*cos( (diffAng*M_PI)/180.f ));

        if(erroP < errorThreshold)
        {
            (*errorScore)+= erroP;
            (*edgeMatch)++;
            i++; j++;
        }else
        {
            (*edgeCut)++;
            if(u[i]->ang < v[j]->ang)
                i++;
            else j++;
        }
    }
    if((*edgeMatch) > 0)
    {
        (*errorScore)/= (*edgeMatch);
    }
}

void GraphMatcher::computeEdgeMatch(vector<GraphLink *> &u, vector<GraphLink *> &v,
                                    vector<PUUPFF> &matchEdges)
{
    // ** We consider that edges are in an increasing order
    sort(u.begin(),u.end(),compRelativeAng);
    sort(v.begin(),v.end(),compRelativeAng);


    unsigned i=0, j=0;

    while( i < u.size() && j < v.size())
    {
        float diffDistance = fabs(u[i]->p - v[j]->p),
              diffAng = fabs(u[i]->rAng - v[j]->rAng);

        if(diffDistance < distThreshold && diffAng < angThreshold)
        {
            matchEdges.push_back(PUUPFF(PUU(i,j), PFF(diffDistance,diffAng)));

            i++; j++;
        }else
        {
            if(u[i]->ang < v[j]->ang)
                i++;
            else j++;
        }
    }
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
float GraphMatcher::computeEdgeMatchAxe(vector<GraphLink *> &u, vector<GraphLink *> &v,
                                       vector<PUUPFF> &matchEdges)
{
    vector<vector<PUU> > match(1);
    unsigned nSingleSolutions = 0, nSolutions=0;

    float meanError;

    for(unsigned i = 0 ; i < u.size() ; i++)
    {
        meanError = findCompativelEdgesByLenght(i,u,v,match[nSingleSolutions]);

//        for(unsigned j = 0 ; j < match[nSingleSolutions].size();j++)
//            cout << match[nSingleSolutions][j].first << " -> " << match[nSingleSolutions][j].second << endl;
//        cout << i << " - mean Error "<< meanError << endl;

        if(meanError < distThreshold)
        {
            nSolutions+=match[nSingleSolutions].size();
            nSingleSolutions++;
            match.push_back(vector<PUU>() );
        }
    }

    vector<PUUPFF> solutions[nSolutions];
    unsigned bestSoluction, bestSize=0, soluctionId=0;
    float bestSoluctionError=99999.9f;

    for(unsigned j = 0 ; j < nSingleSolutions ; j++)
    {
        for(unsigned i = 0 ; i < match[j].size() ; i++)
        {
            float soluctionError= findCompativelEdges(match[j][i].first,
                                                      match[j][i].second,
                                u,v,solutions[soluctionId]);


//            cout << "Soluction "
//                 << match[j][i].first
//                 << " -> "
//                 << match[j][i].second
//                 << " : er "
//                 << soluctionError
//                 << " : size "
//                 << solutions[soluctionId].size()
//                 << endl;

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
//        cout << "Best soluction was "
//             << "error "
//             << bestSoluctionError
//             << " size "
//             << solutions[bestSoluction].size()
//             << endl;
    }
//    cout << "No soluction found!" << endl;

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
float GraphMatcher::findCompativelEdgesByLenght(unsigned sourceEdgeId,
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
float GraphMatcher::findCompativelEdges(unsigned uBeginEdgeId, unsigned vBeginEdgeId,
                                       vector<GraphLink *> &u, vector<GraphLink *> &v,
                                       vector<PUUPFF> &match)
{
    // This method need more than one edge to match
    if(u.size() <= 1 || v.size() <= 1)
        return 999999.9f;

    unsigned uEndEdgeId = (uBeginEdgeId+1)%u.size(),
             vEndEdgeId = (vBeginEdgeId+1)%v.size();
            // Less our two edges that we will analyze (begin and end)
    int uCount =u.size()-2, vCount =v.size()-2;

    float errorAcc=0.f;

    match.push_back(PUUPFF(PUU(uBeginEdgeId,vBeginEdgeId),PFF(-1.f,0.f)));

    // While u and v have at least 2 adj edges to match
    while(uCount >= 0 && vCount >= 0)
    {
        // Compute error between edges
        float uAngDiff = computeEdgeAngDiff(u[uBeginEdgeId],u[uEndEdgeId]),
              vAngDiff = computeEdgeAngDiff(v[vBeginEdgeId],v[vEndEdgeId]),
              angDiff = std::fabs(uAngDiff-vAngDiff),
              error = scaleneError(angDiff,u[uEndEdgeId]->p,v[vEndEdgeId]->p);

        // If current match is not aceptable and we have more edge to test
        if(error > errorThreshold && (vCount > 0 || uCount > 0))
        {
            float   uNextError=error,
                    vNextError=error;

            unsigned uNextId=uEndEdgeId,vNextId=vEndEdgeId;

            int uSearchCount = uCount, vSearchCount = vCount;

            bool vExploreFindMatch=false,
                 uExploreFindMatch=false;

            // if v vertex has a next edge (at least 3 edges)
            if(vCount > 0)
            {
                vExploreFindMatch =
                        exploreBestEdgeMatch(vNextError,uAngDiff,u[uEndEdgeId]->p,
                                     vBeginEdgeId,vNextId,vSearchCount,v);
            }

            if(uCount > 0)
            {
                uExploreFindMatch =
                        exploreBestEdgeMatch(uNextError,vAngDiff,v[vEndEdgeId]->p,
                                     uBeginEdgeId,uNextId,uSearchCount,u);
            }

            // Take explore results
            if(vExploreFindMatch && uExploreFindMatch)
            {
                // If u explore has less error
                if(uNextError < vNextError)
                {   // Take u explore result
                    uCount = uSearchCount;
                    error = uNextError;
                    uEndEdgeId = uNextId;
                }else // take v explore result
                {
                    vCount = vSearchCount;
                    error = vNextError;
                    vEndEdgeId = vNextId;
                }
            }else
            if(vExploreFindMatch)
            {
                vCount = vSearchCount;
                error = vNextError;
                vEndEdgeId = vNextId;
            }else
            if(uExploreFindMatch)
            {
                uCount = uSearchCount;
                error = uNextError;
                uEndEdgeId = uNextId;
            }
        }

        if(error > errorThreshold)
        {
            // No match found
            if(match.size() == 1) // First match
            {
                // Base edge (uBegin,vBegin) not confirmed!
//                cout << "Base edge not confirmed!" << endl;
                match.clear();
            }
            break;
        }

        // Match was found !
        match.push_back(PUUPFF(PUU(uEndEdgeId,vEndEdgeId),PFF(error,0.f)));
        errorAcc+= error;

        uBeginEdgeId = uEndEdgeId;
        vBeginEdgeId = vEndEdgeId;

        uEndEdgeId = (uEndEdgeId+1)%u.size(),
        vEndEdgeId = (vEndEdgeId+1)%v.size(),
         // Less one edge for each vertex who now are used in uEndId and vEndId
         uCount--; vCount--;
    }
    if(match.size() >= 2)
        return errorAcc/(match.size()-1);
    else return 999999.9f;
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
bool GraphMatcher::exploreBestEdgeMatch(float &baseError, float uAngDiff, float uEndLength,
                                        unsigned vBeginId, unsigned &vEndId, int &vCount,
                                        vector<GraphLink*> &v)
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
        nextError = scaleneError(angDiff,uEndLength,v[vNextId]->p);

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

float GraphMatcher::computeEdgeAngDiff(GraphLink *uBegin, GraphLink *uEnd)
{
    float angDiff = fabs(uEnd->ang - uBegin->ang);
    if(angDiff>180.f) angDiff = 360.f - angDiff;
    return angDiff;
}

float GraphMatcher::computeScoreBetweenEdges(GraphLink * uBegin, GraphLink * uEnd,
                                             GraphLink * vBegin, GraphLink * vEnd)
{
    float uAngDiff = fabs(uEnd->ang - uBegin->ang),
          vAngDiff = fabs(vEnd->ang - vBegin->ang);

    if(uAngDiff>180.f) uAngDiff = 360.f - uAngDiff;
    if(vAngDiff>180.f) vAngDiff = 360.f - vAngDiff;

    float angDiff = fabs(uAngDiff-vAngDiff);

    return scaleneError(angDiff,uEnd->p,vEnd->p);
}

void GraphMatcher::findGraphMatchWithInitialGuess(vector<vector<GraphLink *> > &u, vector<vector<GraphLink *> > &v,
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


    while(!q.empty())
    {
        vector<PUUPFF> &initEdgeMatch = initialVertexMatch.edgeMatchInfo;
        for(unsigned i = 0 ; i < initEdgeMatch.size(); i++)
        {
            q.push(PUUPUU(
                       PUU(initialVertexMatch.uID, initEdgeMatch[i].first.first),
                       PUU(initialVertexMatch.vID, initEdgeMatch[i].first.second)
                          ));
        }
    }
}

void GraphMatcher::computeAxeEdgeMatch(vector<GraphLink *> &u, vector<GraphLink *> &v, vector<PUUPFF> &matchEdges)
{
//    // ** We consider that edges are in an increasing order
//    sort(u.begin(),u.end(),compRelativeAng);
//    sort(v.begin(),v.end(),compRelativeAng);

//    sqrt(uP*uP + vP*vP -2.f*uP*vP*cos( (diffAng*M_PI)/180.f ));

//    unsigned i=0, j=0;
//    while( i < u.size() && j < v.size())
//    {
//        float diffDistance = fabs(u[i]->p - v[j]->p),
//              diffAng = fabs(u[i]->rAng - v[j]->rAng);

//        if(diffDistance < distThreshold)
//        {
//            matchEdges.push_back(PUUPFF(PUU(i,j), PFF(diffDistance,diffAng)));
//            i++; j++;
//        }else
//        {
//            if(u[i]->ang < v[j]->ang)
//                i++;
//            else j++;
//        }
//    }
}

void GraphMatcher::computeEdgeMatchEscaleno(vector<GraphLink *> &u, vector<GraphLink *> &v, vector<MatchInfoExtended::PUUPFF> &matchEdges)
{
    // ** We consider that edges are in an increasing order
    sort(u.begin(),u.end(),compRelativeAng);
    sort(v.begin(),v.end(),compRelativeAng);

    unsigned i=0, j=0;

    while( i < u.size() && j < v.size())
    {
        float uP = u[i]->p, vP = v[j]->p,
              diffAng = fabs(u[i]->rAng - v[j]->rAng),
              erroP = sqrt(uP*uP + vP*vP -2.f*uP*vP*cos( (diffAng*M_PI)/180.f ));

        if(erroP < errorThreshold)
        {
            matchEdges.push_back(PUUPFF(PUU(i,j), PFF(erroP,0)));

            i++; j++;
        }else
        {
            if(u[i]->ang < v[j]->ang)
                i++;
            else j++;
        }
    }
}

void GraphMatcher::computeEdgeErrorGula(vector<GraphLink *> &u, vector<GraphLink *> &v,
                                        float *angScore, float *distScore,
                                        unsigned *edgeMatch, unsigned *edgeCut)
{
    if(u.size() == 1 || v.size() == 1)
        return;

    // ** We consider that edges are in an increasing order of inclination (slope)

    unsigned i,j, k,
             uE=u.size(), vE= v.size();

    GraphLink *u1, *u2, *v1, *v2;
    float diffDist1, diffDist2,
          diffAng1, diffAng2, diffAng;

    //= fabs(u[i]->rAng - v[j]->rAng);

    while( i < u.size() && j < v.size())
    {
        u1 = u[i], v1 = v[j];
        diffDist1 = fabs(u1->p - v1->p);

        if(diffDist1 > distThreshold)
        {
            j++; // next v edge
            continue;
        }
        // OK, distance matchs found
        // go to next u,v edge
        i++; j++;

        u2 = u[i], v2 = v[j];

        // Now, compute the ang diff
        // between two pairs of edge (u1,u2) (v1,v2)

        if(u1->ang > u2->ang)
            diffAng1 = (360.f - u1->ang) + u2->ang;
        else diffAng1 = u2->ang - u1->ang;

        if(v1->ang > v2->ang)
            diffAng2 = (360.f - v1->ang) + v2->ang;
        else diffAng2 = v2->ang - v1->ang;

        diffAng = fabs(diffAng1 - diffAng2);

        if(diffAng > angThreshold)
        {
            // Ang diff no match, go to next edge
            if(diffDist1 < diffDist2)
                i++;
            else j++;
            continue;
        }
        // Ang match was found,
        // now comopute the lenght matchs
        diffDist2 = fabs(u2->p - v2->p);
        if(diffDist2 > distThreshold)
        {
            continue;
        }else
        {
            //matchEdges.push_back(PUUPFF(PUU(i,j), PFF(diffDistance,diffAng)));
            i++; j++;
        }

    }
}

bool GraphMatcher::findPairOfVertex(vector<GraphLink*> &u, vector<GraphLink*> &v, float *invAngError, float *edgeDistError, float *angCorrectionMean, int *edgeCut)
{
    // Sort edges by weight increasing order (distances vertex to vertex)
    sort(u.begin(),u.end(), compDist);
    sort(v.begin(),v.end(), compDist);

    int i , j;
    float distError, angError, ACangCorrectionMean=0.f;

    *edgeDistError=0.f;
    *invAngError=0.f;
    edgeCut=0;

    // Compute edges match by wight (vertex distances)
    j = 0;
    i = 0;
    while(i < u.size() && j < v.size())
    {
        distError = fabs(u[i]->p - v[j]->p);

        if(distError > distThreshold)
        {   // cut no match edge

            // cut the edge with lowest weight
            if(v[i]->p < u[j]->p)
            {
                *edgeDistError+= v[i]->p;
                edgeCut++;
                v.erase(v.begin()+j);
            }else
            {
//                edgeDistError+= cu[i]->p;
                edgeCut++;
                u.erase(u.begin()+i);
            }
        }else
        {
            *edgeDistError += distError;
            ACangCorrectionMean+= u[i]->ang - v[j]->ang;
            i++;
            j++;
        }
    }

    while(i < u.size())
    {
        *edgeDistError+= u[i]->p;
        u.erase(u.begin()+i);
    }

    while(j < v.size())
    {
        *edgeDistError+= v[j]->p;
        v.erase(v.begin()+j);
    }

    *angCorrectionMean = ACangCorrectionMean/u.size();

    if(u.size() <= 1 || v.size() <= 1)
    {
        // No match found
        return false; // Infinty
    }

    // Compute vertex match by invAng (Edges distances)
    GraphLink::computeInvAngStatic(v);// No change the edges order, it's important
    GraphLink::computeInvAngStatic(u);// No change the edges order

    for(i = 0; i < v.size(); i++)
    {
        angError = fabs(v[i]->invAngle - u[i]->invAngle);
        if(angError > angThreshold)
        {   // No match found
            return false;
        }
        *invAngError += angError;
    }

    return true;
}

void GraphMatcher::findPairOfVertexByDistance(vector<GraphLink *> &u, vector<GraphLink *> &v, float *edgeAngError, float *edgeDistError, float *angCorrectionMean,unsigned *edgeMatch, unsigned *edgeCut)
{
/*
    Problema desta solucao:
        As vezes a aresta que melhor encaixa é menor
    do que uma aresta que tambem encaixa.
        E se tiver duas aresta de tamanho igual mas
    angulos diferentes ? E só uma delas encaixa e a outra
    não está presente no proximo grafo.
*/

    // Sort edges by weight increasing order (distances vertex to vertex)
    sort(u.begin(),u.end(), compDist);
    sort(v.begin(),v.end(), compDist);

    int i , j;
    float distError, angError, ACangCorrectionMean=0.f;

    *edgeDistError=0.f;
    *edgeAngError=0.f;
    *edgeCut = *edgeMatch =0;

    // Compute edges match by wight (vertex distances)
    j = 0;
    i = 0;
    while(i < u.size() && j < v.size())
    {
        distError = fabs(u[i]->p - v[j]->p);

        if(distError > distThreshold)
        {   // cut no match edge

            // cut the edge with lowest weight
            if(v[i]->p < u[j]->p)
            {
                (*edgeDistError)+= v[i]->p;
                (*edgeCut)++;
                v.erase(v.begin()+j);
            }else
            {
                (*edgeCut)++;
                u.erase(u.begin()+i);
            }
        }else
        {
            (*edgeDistError) += distError;
            ACangCorrectionMean+= u[i]->ang - v[j]->ang;
            i++;
            j++;
        }
    }

    while(i < u.size())
    {
        (*edgeCut)++;
        u.erase(u.begin()+i);
    }

    while(j < v.size())
    {
        (*edgeCut)++;
        v.erase(v.begin()+j);
    }

    *angCorrectionMean = ACangCorrectionMean/u.size();

    if(u.size() <= 1 || v.size() <= 1)
    {
        // No match found
        return;
    }

    // Compute vertex match by invAng (Edges distances)
    GraphLink::computeInvAng(v);// Change the edges order, it's important
    GraphLink::computeInvAng(u);// Change the edges order

    for(i = 0; i < v.size(); i++)
    {
        angError = fabs(v[i]->invAngle - u[i]->invAngle);
        distError = fabs(v[i]->p - u[i]->p);
        if(angError <= angThreshold &&
           distError <= distThreshold )
        {   // Match found
            (*edgeMatch)++;
            *edgeAngError += angError;
        }else
        {
            (*edgeCut)+= v.size();
            (*edgeMatch) =0;
            return;
        }
    }
    *edgeAngError/= *edgeMatch;

}

void GraphMatcher::compVertexDist(const vector<GraphLink *> &u, const vector<GraphLink *> &v)
{
/*
    const vector<GraphLink *> *vm , *vM;
    if(u.size() > v.size())
    {
        vM = &u;
        vm = &v;
    }else
    {
        vM = &v;
        vm = &u;
    }

    vector< vector< pair<float , unsigned> > > match(vm->size());
    float distError;
    unsigned i, j;
    for(i = 0 ; i < vm->size() ; i++)
    {
        for(j = 0 ; j < vM->size(); j++)
        {
            distError = fabs((*vm)[i]->p - (*vM)[j]->p);
            if( distError < distThreshold)
                match[i].push_back(make_pair(distError, j));
        }
        sort(match[i].begin(), match[i].end());
    }

    for(i = 0; i < match.size() ; i++)
    {
        for(j = 0 ; j < match[i].size() ; j++)
        {
            cout << "Match " << i << " com " << j << " score " << distError << endl;
        }
    }
*/
}

void GraphMatcher::buscaMatch(GraphLink *link, vector<GraphLink *> vertex, int star, int end, float ang)
{
/*
    float angError = fabs(link->invAngle - vertex[star]->invAngle),
          distError = fabs(link->p - vertex[star]->p);

    if(angError < angThreshold)
    {
        if

    }
*/
}

void GraphMatcher::findVerticesMatchByAngDiff(vector<GraphLink *> &u, vector<GraphLink *> &v)
{
//    // we consider that edges are sorted in ang inclination orden
//    unsigned i , j,
//            uE_end,vE_end,
//            uE_n=u.size(), vE_n=v.size(),
//            edgeMatch=0;

//    float angDiff,
//          avgAngDiff=0.f;

//    GraphLink *ue, *ve;

//    pair<
//        float, // average ang diff
//        vector< pair<unsigned,unsigned> > // edge match
//        > matchs;

//    i = 0; j =k;
//    uE_end = uE_n;
//    if(k == 0)
//        vE_end = vE_n;
//    else vE_end = k;

//    while(i < uE_end && j < vE_end)
//    {
//        ue = u[i], ve = v[j];

//        angDiff = fabs(ue->invAngle - ve->invAngle);
//        if(angDiff > angThreshold)
//        {
//            if(ue->invAngle < ve->invAngle)
//            {
//                i = (i+1)%uE_n;
//            }else j = (j+1)%vE_n;
//            continue;
//        }

//        // Ang match (i,j)
//        matchs.second.push_back(PUU(i,j));
//        avgAngDiff+= angDiff;

//        // Go to next edge
//        i = (i+1)%uE_n;
//        j = (j+1)%vE_n;
//    }

}

void GraphMatcher::findBestEdgeMatch(vector<GraphLink *> &u, vector<GraphLink *> &v, unsigned ui, unsigned vi, unsigned &uj, unsigned &vj)
{
//    unsigned i,j,
//            uE_end,vE_end,
//            uE_n=u.size(), vE_n=v.size();

//    float angDiff,
//          avgAngDiff=0.f;

//    GraphLink *ue, *ve;

//    pair<
//        float, // average ang diff
//        vector< pair<unsigned,unsigned> > // edge match
//        > matchs;

//    uj=ui+1,vj=vi+1;
//    i = 0; j =k;
//    uE_end = uE_n;
//    if(k == 0)
//        vE_end = vE_n;
//    else vE_end = k;

//    while(i < uE_end && j < vE_end)
//    {
//        ue = u[i], ve = v[j];

//        angDiff = fabs(ue->invAngle - ve->invAngle);
//        if(angDiff > angThreshold)
//        {
//            if(ue->invAngle < ve->invAngle)
//            {
//                i = (i+1)%uE_n;
//            }else j = (j+1)%vE_n;
//            continue;
//        }

//        // Ang match (i,j)
//        matchs.second.push_back(PUU(i,j));
//        avgAngDiff+= angDiff;

//        // Go to next edge
//        i = (i+1)%uE_n;
//        j = (j+1)%vE_n;
//    }
}

void GraphMatcher::byDistanceCompare(vector<GraphLink *> &u, vector<GraphLink *> &v)
{
    vector<float> bipartiteEdgeMatch[u.size()];

    for(unsigned i = 0; i < u.size() ;i++)
        bipartiteEdgeMatch[i].resize(v.size(),-1);

    distThreshold = 5;

    for(unsigned i = 0; i < u.size() ;i++)
    {
        for(unsigned j = 0; j < v.size() ;j++)
        {
            float diff = fabs(u[i]->p - v[j]->p);
            if(diff <= distThreshold)
            {
                bipartiteEdgeMatch[i][j] = diff;
            }
        }
    }

    cout << "Bipartite Graph" << endl;
    for(unsigned i = 0; i < u.size() ;i++)
    {
        for(unsigned j = 0; j < v.size() ;j++)
        {
            if(bipartiteEdgeMatch[i][j] >= 0)
                cout << i+1 << " <-(" << bipartiteEdgeMatch[i][j] << ")-> " << j+1 << endl;
        }
    }

//    vector<PUU> edgeMatchGraph[u.size()];

//    for(unsigned m = 0; m < u.size() ;m++)
//    {
//        for(unsigned n = 0; n < u.size() ;n++)
//        {
//            for(unsigned i = m+1; i != m;i= (i+1)%u.size())
//            {
//                for(unsigned j = n+1; j != n ;j = (j+1)%v.size())
//                {
//                    if(bipartiteEdgeMatch[i][j] >= 0)
//                        cout << i+1 << " <-(" << bipartiteEdgeMatch[i][j] << ")-> " << j+1 << endl;
//                }
//            }
//        }
//    }

    byDistanceCompare_(u,v,bipartiteEdgeMatch);
}

void GraphMatcher::byDistanceCompare_(vector<GraphLink *> &u, vector<GraphLink *> &v,
                                      vector<float> *edgeMatch)
{
//    unsigned uIni=0, vIni=0;


//    for(unsigned i = uIni ; i < u.size() ; i++)
//    {
//        for(unsigned j = vIni; j < v.size() ; j++)
//        {
//            if(edgeMatch)


//        }
//    }
}

void GraphMatcher::byDistanceCompare_rec(vector<GraphLink *> &u, unsigned uIni,
                                         vector<GraphLink *> &v, unsigned vIni,
                                         vector<float> *edgeMatch)
{
    for(unsigned i = uIni ; i < u.size() ; i++)
    {
        for(unsigned j = vIni; j < v.size() ; j++)
        {
//            if()
        }
    }
}

float GraphMatcher::diffAngle(vector<GraphLink *> &u, unsigned fromEdgeID, unsigned toEdgeID)
{
    unsigned numOfEdge = u.size();

    if(fromEdgeID >= numOfEdge || toEdgeID >= numOfEdge)
    {
        cout << "Erro when computing edge ang diff, invalid edges IDs" << endl;
        return 0.f;
    }

    float angDiff = 0.f;

    for(unsigned i = fromEdgeID; i != toEdgeID ; i = (i+1)%numOfEdge)
    {
        angDiff += u[i]->invAngle;
    }
    return angDiff;
}

float GraphMatcher::scaleneError(float degreeAngDiff, float length1, float length2)
{
    return sqrt(length1*length1 + length2*length2 -2.f*length1*length2*cos( (degreeAngDiff*M_PI)/180.f ));
}

GraphMatcher::GraphMatcher(const SonarConfig &config)
{
    /*
    distThreshold = config.graphMatcher_distThreshold;
    angThreshold = config.graphMatcher_angThreshold;
    stdXWeight = config.graphMatcher_stdXWeightError;
    stdYWeight = config.graphMatcher_stdYWeightError;
    stdIWeight = config.graphMatcher_stdIWeightError;
    angWeight = config.graphMatcher_angWeightError;
    invAngWeight = config.graphMatcher_invAngWeightError;
    edgeDistanceWeightError= config.graphMatcher_edgeDistanceWeightError;
    */

    drawVertexMatching= config.graphMatcher_drawVertexMatching;

    distThreshold = 5.f;
    angThreshold = 15.f;
    errorThreshold = 30.f;

    stdXWeight = 1.f;
    stdYWeight = 1.f;
    stdIWeight = 0.f;
    meanIWeight = 1.f;
    angWeight = 1.f;

    edgeAngWeight = 25.0f;
    edgeDistWeight = 3.0f;
    edgeCutWeight = 0.f;
    edgeMatchWeight = -15.f;

    edgeWeight = 1.f;
    vertexWeight = 0.f;

    SIFTCutValue = 5.f;

    minSimilarEdgeToMatch = 4;
}

void GraphMatcher::matche(SonarDescritor *sd)
{
}

void GraphMatcher::matche(SonarDescritor *sd1, SonarDescritor *sd2,
                          vector< pair<unsigned, unsigned> > &vertexMatch)
{

    vector< vector<GraphLink*> > &g1 = sd1->graph , &g2 = sd2->graph;

    float score, bestScore = FLT_MAX,
          edgeScore=0.f, vertexScore,
          angCorrection=0.f,
          edgeDistError=0.f, edgeAngError=0.f;

    int vertexIDMatch=-1;

    unsigned edgeCut=0, edgeMatch=0;

    // Foreach vertex u from 1st graph
    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        // With more than 1 edge
        if(g1[u].size() <= 1) continue;

        bestScore = FLT_MAX;
        vertexIDMatch=-1;

        // Foreach vertex v from 2nd graph
        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            // With more than 1 edge
            if(g2[v].size() <= 1) continue;

            // Compute Edge Error
            computeEdgeError(g1[u], g2[v],
                             &edgeAngError,&edgeDistError,
                             &edgeMatch, &edgeCut);

//            if(edgeMatch < edgeCut) continue;

            edgeScore = edgeAngError*edgeAngWeight +
                        edgeDistError*edgeDistWeight +
                        edgeCut * edgeCutWeight + edgeMatch * edgeMatchWeight;

            //Compute the vertex Error
            vertexScore = fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight +
                    fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight +
                    fabs(sd1->gaussians[u].di-sd2->gaussians[v].di) * stdIWeight +
                    fabs(sd1->gaussians[u].intensity-sd2->gaussians[v].intensity) * meanIWeight +
                    fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight;
                    ;

            score = edgeScore*edgeWeight + vertexScore*vertexWeight;

            if(bestScore > score)
            {
                bestScore = score;
                vertexIDMatch = v;
                angCorrection = edgeAngError;
            }

            if(drawVertexMatching)
            { /*
                int w=1800 , h=900;
                Mat colorImg = Mat::zeros(Size(w,h),CV_8UC3);
                drawVertexMatch(colorImg,u,sd1->gaussians[u],g1[u],v,sd2->gaussians[v],g2[v],Rect(0,0,w,h/2));
                drawVertexMatch(colorImg,u,sd1->gaussians[u],cu,v,sd2->gaussians[v],cv,Rect(0,h/2,w,h/2));
                imshow("VertexMatch",colorImg);
                waitKey();
              */
            }
        }

        if(vertexIDMatch >=0)
        {
            vertexMatch.push_back(pair<unsigned,unsigned>(u,vertexIDMatch));
            cout << "Match " << u << " com " << vertexIDMatch << " score " << bestScore
                 << " AngCorrection " << angCorrection
                 << " translacao = " << sd1->gaussians[u].x - sd2->gaussians[vertexIDMatch].x
                 << " , " << sd1->gaussians[u].y - sd2->gaussians[vertexIDMatch].y << endl;
        }
    }
}

void GraphMatcher::matcheSIFTCut(SonarDescritor *sd1, SonarDescritor *sd2,
                                 vector<MatchInfo> &vertexMatch)
{
    vector< vector<GraphLink*> > &g1 = sd1->graph,
                                 &g2 = sd2->graph;

    typedef pair<float,int> PFI;
    typedef pair< PFI , float> PFIF;

    vector<PFIF> invScore(g2.size(), PFIF(PFI(FLT_MAX,-1) ,FLT_MAX));
    // invScore[ destID ] ( ( bestScore , srcIDBest) , SecondBestScore)

    int matchInfo;

    float score, bestScore = FLT_MAX, secondBestScore,
          edgeScore=0.f, vertexScore,
          angCorrection=0.f,
          edgeDistError=0.f, edgeAngError=0.f;

    int vertexIDMatch=-1;

    unsigned edgeCut=0, edgeMatch=0;

    // For each gaussian u in img1
    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1[u].size() < minSimilarEdgeToMatch) continue;

        bestScore = secondBestScore = FLT_MAX;
        vertexIDMatch=-1;

        // For each gaussian v in img2
        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2[v].size() < minSimilarEdgeToMatch) continue;

            // Compute Edge Error
//            computeEdgeError(g1[u], g2[v],
//                             &edgeAngError,&edgeDistError,
//                             &edgeMatch, &edgeCut);

//            computeEdgeErrorEscaleno(g1[u], g2[v],
//                             &edgeScore,
//                             &edgeMatch, &edgeCut);

            vector<PUUPFF> matchEdges;
            edgeScore = computeEdgeMatchAxe(g1[u], g2[v],
                                            matchEdges);
            edgeMatch = matchEdges.size();

            if(edgeMatch < minSimilarEdgeToMatch) continue;

//            edgeScore = edgeAngError*edgeAngWeight
//                        + edgeDistError*edgeDistWeight
//                        + edgeCut * edgeCutWeight
//                      + edgeMatch * edgeMatchWeight
                    ;

            //Compute the vertex Error
//            vertexScore = fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight +
//                    fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight +
//                    fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight +
//                    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight +
//                    fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight;
                    ;

//            score = edgeScore*edgeWeight + vertexScore*vertexWeight;
            score = edgeScore;
/*
            cout << '\t' << u << " -> " << v << " : " << score
                 << endl << "\t\t"
                 <<  " edgeScore: " << edgeScore*edgeWeight << endl << "\t\t"
            << " Ang " << edgeAngError*edgeAngWeight
            << " Dist " << edgeDistError*edgeDistWeight
            << " Cut " << edgeCut * edgeCutWeight
            << " Match " << edgeMatch * edgeMatchWeight
                 << endl << "\t\t"
                 << " vertexScore: " << vertexScore*vertexWeight << endl << "\t\t"
            << " stdx " << fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight
            << " stdy " << fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight
            << " stdI " << fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight
            << " meanI " <<    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight
            << " ang " << fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight
                 << endl;
*/
            if(bestScore > score)
            {
                secondBestScore = bestScore;
                bestScore = score;
                vertexIDMatch = v;
                angCorrection = edgeAngError;
            }else if(secondBestScore > score)
            {
                secondBestScore = score;
            }

            if(drawVertexMatching)
            { /*
                int w=1800 , h=900;
                Mat colorImg = Mat::zeros(Size(w,h),CV_8UC3);
                drawVertexMatch(colorImg,u,sd1->gaussians[u],g1[u],v,sd2->gaussians[v],g2[v],Rect(0,0,w,h/2));
                drawVertexMatch(colorImg,u,sd1->gaussians[u],cu,v,sd2->gaussians[v],cv,Rect(0,h/2,w,h/2));
                imshow("VertexMatch",colorImg);
                waitKey();
              */
            }
        }

//        cout << u << " -> " << bestScore << " , " << secondBestScore << endl;

        // If was match
        if(bestScore < FLT_MAX)
        {
            // If was two matchs
            if(secondBestScore < FLT_MAX &&
               secondBestScore - bestScore < SIFTCutValue)
            { // If second match is too close best match, don't take this match
                continue;
            }

            if(invScore[vertexIDMatch].first.first > bestScore)
            {
                // Now the old best score is second best score
                invScore[vertexIDMatch].second = invScore[vertexIDMatch].first.first;

                // Best score is the actual the best!
                invScore[vertexIDMatch].first.first = bestScore;
                invScore[vertexIDMatch].first.second = u;

            }else if(invScore[vertexIDMatch].second > bestScore )
            {
                // Update the second best score
                invScore[vertexIDMatch].second = bestScore;
            }

//            vertexMatch.push_back(pair<unsigned , unsigned>(u,vertexIDMatch));
//            cout << "Match " << u << " com " << vertexIDMatch
//                 << " score " << bestScore << " second score " << secondBestScore << endl;
        }
    }

    // Cuting week matchs
    for(unsigned i =0  ; i < g2.size() ; i++)
    {
        if(invScore[i].first.first < FLT_MAX)
        {
           if(invScore[i].second - invScore[i].first.first < SIFTCutValue)
            continue;

            vertexMatch.push_back(MatchInfo(invScore[i].first.second,i));

//            cout << "Match " << invScore[i].first.second << " com " << i
//                 << " score " << invScore[i].first.first << " second score " << invScore[i].second << endl;
        }
    }
}

void GraphMatcher::matcheScalene(SonarDescritor *sd1, SonarDescritor *sd2,
                                 vector<MatchInfoWeighted> &vertexMatch)
{
    vector< vector<GraphLink*> > &g1 = sd1->graph,
                                 &g2 = sd2->graph;

    typedef pair<float,int> PFI;
    typedef pair< PFI , float> PFIF;

    vector<PFIF> invScore(g2.size(), PFIF(PFI(FLT_MAX,-1) ,FLT_MAX));
    // invScore[ destID ] ( ( bestScore , srcIDBest) , SecondBestScore)

    int matchInfo;

    float score, bestScore = FLT_MAX, secondBestScore,
          edgeScore=0.f, vertexScore,
          angCorrection=0.f,
          edgeDistError=0.f, edgeAngError=0.f;

    int vertexIDMatch=-1;

    unsigned edgeCut=0, edgeMatch=0;

    // For each gaussian u in img1
    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1[u].size() < minSimilarEdgeToMatch) continue;

        bestScore = secondBestScore = FLT_MAX;
        vertexIDMatch=-1;

        // For each gaussian v in img2
        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2[v].size() < minSimilarEdgeToMatch) continue;

            // Compute Edge Error
//            computeEdgeError(g1[u], g2[v],
//                             &edgeAngError,&edgeDistError,
//                             &edgeMatch, &edgeCut);

            computeEdgeErrorEscaleno(g1[u], g2[v],
                             &edgeScore,
                             &edgeMatch, &edgeCut);

            if(edgeMatch < minSimilarEdgeToMatch) continue;

//            edgeScore = edgeAngError*edgeAngWeight
//                        + edgeDistError*edgeDistWeight
//                        + edgeCut * edgeCutWeight
//                      + edgeMatch * edgeMatchWeight
                    ;

            //Compute the vertex Error
//            vertexScore = fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight +
//                    fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight +
//                    fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight +
//                    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight +
//                    fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight;
                    ;

//            score = edgeScore*edgeWeight + vertexScore*vertexWeight;
            vertexMatch.push_back(MatchInfoWeighted(u,v,edgeScore));
/*
            cout << '\t' << u << " -> " << v << " : " << score
                 << endl << "\t\t"
                 <<  " edgeScore: " << edgeScore*edgeWeight << endl << "\t\t"
            << " Ang " << edgeAngError*edgeAngWeight
            << " Dist " << edgeDistError*edgeDistWeight
            << " Cut " << edgeCut * edgeCutWeight
            << " Match " << edgeMatch * edgeMatchWeight
                 << endl << "\t\t"
                 << " vertexScore: " << vertexScore*vertexWeight << endl << "\t\t"
            << " stdx " << fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight
            << " stdy " << fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight
            << " stdI " << fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight
            << " meanI " <<    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight
            << " ang " << fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight
                 << endl;
*/
        }
    }
}

void GraphMatcher::SiftCut(vector<MatchInfoWeighted> &vertexMatch,
                           unsigned guSize, unsigned gvSize)
{

}

void GraphMatcher::matcheSIFTCutDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                                      vector<pair<unsigned, unsigned> > &vertexMatch,
                                      vector<MatchInfoExtended> &matchInfo)
{
    vector< vector<GraphLink*> > &g1 = sd1->graph,
                                 &g2 = sd2->graph;

    typedef pair<float,int> PFI;
    typedef pair< PFI , float> PFIF;

    vector<PFIF> invScore(g2.size(), PFIF(PFI(FLT_MAX,-1) ,FLT_MAX));
    // invScore[ destID ] ( ( bestScore , srcIDBest) , SecondBestScore)

    float score, bestScore = FLT_MAX, secondBestScore,
          edgeScore=0.f, vertexScore,
          angCorrection=0.f,
          edgeDistError=0.f, edgeAngError=0.f;

    int vertexIDMatch=-1;

    unsigned edgeCut=0, edgeMatch=0;

    // For each gaussian u in img1
    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1[u].size() < minSimilarEdgeToMatch) continue;

        bestScore = secondBestScore = FLT_MAX;
        vertexIDMatch=-1;

        // For each gaussian v in img2
        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2[v].size() < minSimilarEdgeToMatch) continue;

            // Compute Edge Error
//            computeEdgeError(g1[u], g2[v],
//                             &edgeAngError,&edgeDistError,
//                             &edgeMatch, &edgeCut);

//            computeAxeEdgeMatch(g1[u], g2[v],
//                             &edgeScore,
//                             &edgeMatch, &edgeCut);

            computeEdgeErrorEscaleno(g1[u], g2[v],
                             &edgeScore,
                             &edgeMatch, &edgeCut);

            if(edgeMatch < minSimilarEdgeToMatch) continue;

//            edgeScore = edgeAngError*edgeAngWeight
//                        + edgeDistError*edgeDistWeight
//                        + edgeCut * edgeCutWeight
//                      + edgeMatch * edgeMatchWeight
                    ;

            //Compute the vertex Error
//            vertexScore = fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight +
//                    fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight +
//                    fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight +
//                    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight +
//                    fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight;
                    ;

//            score = edgeScore*edgeWeight + vertexScore*vertexWeight;
            score = edgeScore;
/*
            cout << '\t' << u << " -> " << v << " : " << score
                 << endl << "\t\t"
                 <<  " edgeScore: " << edgeScore*edgeWeight << endl << "\t\t"
            << " Ang " << edgeAngError*edgeAngWeight
            << " Dist " << edgeDistError*edgeDistWeight
            << " Cut " << edgeCut * edgeCutWeight
            << " Match " << edgeMatch * edgeMatchWeight
                 << endl << "\t\t"
                 << " vertexScore: " << vertexScore*vertexWeight << endl << "\t\t"
            << " stdx " << fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight
            << " stdy " << fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight
            << " stdI " << fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight
            << " meanI " <<    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight
            << " ang " << fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight
                 << endl;
*/
            if(bestScore > score)
            {
                secondBestScore = bestScore;
                bestScore = score;
                vertexIDMatch = v;
                angCorrection = edgeAngError;
            }else if(secondBestScore > score)
            {
                secondBestScore = score;
            }

            if(drawVertexMatching)
            { /*
                int w=1800 , h=900;
                Mat colorImg = Mat::zeros(Size(w,h),CV_8UC3);
                drawVertexMatch(colorImg,u,sd1->gaussians[u],g1[u],v,sd2->gaussians[v],g2[v],Rect(0,0,w,h/2));
                drawVertexMatch(colorImg,u,sd1->gaussians[u],cu,v,sd2->gaussians[v],cv,Rect(0,h/2,w,h/2));
                imshow("VertexMatch",colorImg);
                waitKey();
              */
            }
        }

//        cout << u << " -> " << bestScore << " , " << secondBestScore << endl;

        // If was match
        if(bestScore < FLT_MAX)
        {
            // If was two matchs
            if(secondBestScore < FLT_MAX &&
               secondBestScore - bestScore < SIFTCutValue)
            { // If second match is too close best match, don't take this match
                continue;
            }

            if(invScore[vertexIDMatch].first.first > bestScore)
            {
                // Now the old best score is second best score
                invScore[vertexIDMatch].second = invScore[vertexIDMatch].first.first;

                // Best score is the actual the best!
                invScore[vertexIDMatch].first.first = bestScore;
                invScore[vertexIDMatch].first.second = u;

            }else if(invScore[vertexIDMatch].second > bestScore )
            {
                // Update the second best score
                invScore[vertexIDMatch].second = bestScore;
            }

//            vertexMatch.push_back(pair<unsigned , unsigned>(u,vertexIDMatch));
//            cout << "Match " << u << " com " << vertexIDMatch
//                 << " score " << bestScore << " second score " << secondBestScore << endl;
        }
    }

    // Cuting week matchs
    matchInfo.clear();
    for(unsigned i =0  ; i < g2.size() ; i++)
    {
        if(invScore[i].first.first < FLT_MAX)
        {
           if(invScore[i].second - invScore[i].first.first < SIFTCutValue)
            continue;

           matchInfo.push_back(MatchInfoExtended());
           MatchInfoExtended &info = matchInfo.back();

           info.bestScore = invScore[i].first.first;
           info.sBestScore = invScore[i].second;

           info.uID = invScore[i].first.second;
           info.vID = i;

//           computeEdgeMatch(g1[info.uID], g2[info.vID],
//                            info.edgeMatchInfo);
           computeEdgeMatchEscaleno(g1[info.uID], g2[info.vID],
                            info.edgeMatchInfo);

            vertexMatch.push_back(pair<unsigned , unsigned>(invScore[i].first.second,i));

//            cout << "Match " << invScore[i].first.second << " com " << i
//                 << " score " << invScore[i].first.first << " second score " << invScore[i].second << endl;
        }
    }
}

void GraphMatcher::computeCompleteMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                                        vector<vector<MatchInfoExtended> > &match)
{
//    vector< vector<GraphLink*> > &g1 = sd1->graph,
//                                 &g2 = sd2->graph;

//    match.clear();
//    match.resize(g1.size(), vector<vector<MatchInfoExtended> >(g2.size()) );

//    // For each gaussian u in img1
//    for(unsigned u = 0 ; u < g1.size() ; u++)
//    {
//        if(g1[u].size() < minSimilarEdgeToMatch && g1[u].size() < 2)
//            continue;

//        bestScore = secondBestScore = FLT_MAX;
//        vertexIDMatch=-1;

//        // For each gaussian v in img2
//        for(unsigned v = 0 ; v < g2.size(); v++)
//        {
//            if(g2[v].size() < minSimilarEdgeToMatch && g2[v].size() < 2)
//                continue;

//            MatchInfoExtended &iMatch = match[u][v];

//            computeEdgeMatchAxe(g1[u], g2[v],iMatch.edgeMatchInfo);
//        }

////        cout << u << " -> " << bestScore << " , " << secondBestScore << endl;

//        // If was match
//        if(bestScore < FLT_MAX)
//        {
//            // If was two matchs
//            if(secondBestScore < FLT_MAX &&
//               secondBestScore - bestScore < SIFTCutValue)
//            { // If second match is too close best match, don't take this match
//                continue;
//            }

//            if(invScore[vertexIDMatch].first.first > bestScore)
//            {
//                // Now the old best score is second best score
//                invScore[vertexIDMatch].second = invScore[vertexIDMatch].first.first;

//                // Best score is the actual the best!
//                invScore[vertexIDMatch].first.first = bestScore;
//                invScore[vertexIDMatch].first.second = u;

//            }else if(invScore[vertexIDMatch].second > bestScore )
//            {
//                // Update the second best score
//                invScore[vertexIDMatch].second = bestScore;
//            }

////            vertexMatch.push_back(pair<unsigned , unsigned>(u,vertexIDMatch));
////            cout << "Match " << u << " com " << vertexIDMatch
////                 << " score " << bestScore << " second score " << secondBestScore << endl;
//        }
//    }

//    // Cuting week matchs
//    matchInfo.clear();
//    for(unsigned i =0  ; i < g2.size() ; i++)
//    {
//        if(invScore[i].first.first < FLT_MAX)
//        {
//           if(invScore[i].second - invScore[i].first.first < SIFTCutValue)
//            continue;

//           matchInfo.push_back(MatchInfoExtended());
//           MatchInfoExtended &info = matchInfo.back();

//           info.bestScore = invScore[i].first.first;
//           info.sBestScore = invScore[i].second;

//           info.uID = invScore[i].first.second;
//           info.vID = i;

////           computeEdgeMatch(g1[info.uID], g2[info.vID],
////                            info.edgeMatchInfo);
//           computeEdgeMatchEscaleno(g1[info.uID], g2[info.vID],
//                            info.edgeMatchInfo);

//            vertexMatch.push_back(pair<unsigned , unsigned>(invScore[i].first.second,i));

////            cout << "Match " << invScore[i].first.second << " com " << i
////                 << " score " << invScore[i].first.first << " second score " << invScore[i].second << endl;
//        }
//    }
}

void GraphMatcher::NEWMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                            vector<pair<unsigned, unsigned> > &vertexMatch,
                            vector<MatchInfoExtended> &matchInfo)
{
    // Take descripton  graphs
    vector< vector<GraphLink*> > &g1 = sd1->graph,
                                 &g2 = sd2->graph;

    // Graph Match from sd1 to sd2 and from sd2 to sd 1
    vector< vector< pair<float,int> > > graphMatch1to2,
                                        graphMatch2to1;

    // graphMatch[src vertexID
    typedef pair<float,int> PFI;
    typedef pair< PFI , float> PFIF;

    vector<PFIF> invScore(g2.size(), PFIF(PFI(FLT_MAX,-1) ,FLT_MAX));
    // invScore[ destID ] ( ( bestScore , srcIDBest) , SecondBestScore)

    float score, // current score
          bestScore = FLT_MAX, // best score
          secondBestScore, // second best score
          edgeScore=0.f,
          angCorrection=0.f,
          edgeAngError=0.f;

    int vertexIDMatch=-1;

    unsigned edgeCut=0, edgeMatch=0;

    // For each gaussian u in img1
    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1[u].size() < minSimilarEdgeToMatch) continue;

        bestScore = secondBestScore = FLT_MAX;
        vertexIDMatch=-1;

        // For each gaussian v in img2
        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2[v].size() < minSimilarEdgeToMatch) continue;

            // Compute Edge Error
//            computeEdgeError(g1[u], g2[v],
//                             &edgeAngError,&edgeDistError,
//                             &edgeMatch, &edgeCut);

            computeEdgeErrorEscaleno(g1[u], g2[v],
                             &edgeScore,
                             &edgeMatch, &edgeCut);

            if(edgeMatch < minSimilarEdgeToMatch) continue;

//            edgeScore = edgeAngError*edgeAngWeight
//                        + edgeDistError*edgeDistWeight
//                        + edgeCut * edgeCutWeight
//                      + edgeMatch * edgeMatchWeight
                    ;

            //Compute the vertex Error
//            vertexScore = fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight +
//                    fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight +
//                    fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight +
//                    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight +
//                    fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight;
                    ;

//            score = edgeScore*edgeWeight + vertexScore*vertexWeight;
            score = edgeScore;

            if(bestScore > score)
            {
                secondBestScore = bestScore;
                bestScore = score;
                vertexIDMatch = v;
                angCorrection = edgeAngError;
            }else if(secondBestScore > score)
            {
                secondBestScore = score;
            }

            if(drawVertexMatching)
            { /*
                int w=1800 , h=900;
                Mat colorImg = Mat::zeros(Size(w,h),CV_8UC3);
                drawVertexMatch(colorImg,u,sd1->gaussians[u],g1[u],v,sd2->gaussians[v],g2[v],Rect(0,0,w,h/2));
                drawVertexMatch(colorImg,u,sd1->gaussians[u],cu,v,sd2->gaussians[v],cv,Rect(0,h/2,w,h/2));
                imshow("VertexMatch",colorImg);
                waitKey();
              */
            }
        }

//        cout << u << " -> " << bestScore << " , " << secondBestScore << endl;

        // If was match
        if(bestScore < FLT_MAX)
        {
            // If was two matchs
            if(secondBestScore < FLT_MAX &&
               secondBestScore - bestScore < SIFTCutValue)
            { // If second match is too close best match, don't take this match
                continue;
            }

            if(invScore[vertexIDMatch].first.first > bestScore)
            {
                // Now the old best score is second best score
                invScore[vertexIDMatch].second = invScore[vertexIDMatch].first.first;

                // Best score is the actual the best!
                invScore[vertexIDMatch].first.first = bestScore;
                invScore[vertexIDMatch].first.second = u;

            }else if(invScore[vertexIDMatch].second > bestScore )
            {
                // Update the second best score
                invScore[vertexIDMatch].second = bestScore;
            }

//            vertexMatch.push_back(pair<unsigned , unsigned>(u,vertexIDMatch));
//            cout << "Match " << u << " com " << vertexIDMatch
//                 << " score " << bestScore << " second score " << secondBestScore << endl;
        }
    }

    // Cuting week matchs
    matchInfo.clear();
    for(unsigned i =0  ; i < g2.size() ; i++)
    {
        if(invScore[i].first.first < FLT_MAX)
        {
           if(invScore[i].second - invScore[i].first.first < SIFTCutValue)
            continue;

           matchInfo.push_back(MatchInfoExtended());
           MatchInfoExtended &info = matchInfo.back();

           info.bestScore = invScore[i].first.first;
           info.sBestScore = invScore[i].second;

           info.uID = invScore[i].first.second;
           info.vID = i;

//           computeEdgeMatch(g1[info.uID], g2[info.vID],
//                            info.edgeMatchInfo);
           computeEdgeMatchEscaleno(g1[info.uID], g2[info.vID],
                            info.edgeMatchInfo);

            vertexMatch.push_back(pair<unsigned , unsigned>(invScore[i].first.second,i));

//            cout << "Match " << invScore[i].first.second << " com " << i
//                 << " score " << invScore[i].first.first << " second score " << invScore[i].second << endl;
        }
    }
}

void GraphMatcher::matcheM(SonarDescritor *sd1, SonarDescritor *sd2,
                           vector<MatchInfo> &vertexMatch)
{
    vector< vector<GraphLink*> > &g1 = sd1->graph , &g2 = sd2->graph;

    typedef pair<float,int> PFI;
    typedef pair< PFI , float> PFIF;
    vector<PFIF> invScore(g2.size(), PFIF(PFI(FLT_MAX,-1),FLT_MAX));

    vector<GraphLink*> vu, vv;

    float score, bestScore = FLT_MAX,secondBestScore,
          edgeScore=0.f, vertexScore,
          angCorrection=0.f,
          edgeDistError=0.f, edgeAngError=0.f;

    int vertexIDMatch=-1;

    unsigned edgeCut=0, edgeMatch=0;

    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1[u].size() < 3) continue;
        bestScore = secondBestScore = FLT_MAX;
        vertexIDMatch=-1;

        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2[v].size() < 3) continue;

            dinamicCopyVertex(g1[u], vu);
            dinamicCopyVertex(g2[v], vv);

            // Compute Edge Error
            findPairOfVertexByDistance(vu,vv,
                             &edgeAngError,&edgeDistError,
                             &angCorrection,
                             &edgeMatch, &edgeCut);

            if(drawVertexMatching && edgeMatch >= 3)
            {
                int w=1800 , h=900;
                Mat colorImg = Mat::zeros(Size(w,h),CV_8UC3);
                drawVertexMatch(colorImg,u,sd1->gaussians[u],g1[u],v,sd2->gaussians[v],g2[v],Rect(0,0,w,h/2));
                drawVertexMatch(colorImg,u,sd1->gaussians[u],vu,v,sd2->gaussians[v],vv,Rect(0,h/2,w,h/2));
                imshow("VertexMatch",colorImg);
                waitKey();
            }

            freeVertex(vu);
            freeVertex(vv);

            if(edgeMatch < 3) continue;

            edgeScore = edgeAngError*edgeAngWeight
                        + edgeDistError*edgeDistWeight
//                        + edgeCut * edgeCutWeight
//                      + edgeMatch * edgeMatchWeight
                    ;

            //Compute the vertex Error
            vertexScore = fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight +
                    fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight +
                    fabs(sd1->gaussians[u].di-sd2->gaussians[v].di) * stdIWeight +
                    fabs(sd1->gaussians[u].intensity-sd2->gaussians[v].intensity) * meanIWeight +
                    fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight;
                    ;

            score = edgeScore*edgeWeight + vertexScore*vertexWeight;

            cout << '\t' << u << " -> " << v << " : " << score
                 << endl << "\t\t"
                 <<  " edgeScore: " << edgeScore*edgeWeight << endl << "\t\t"
            << " Ang " << edgeAngError*edgeAngWeight
            << " Dist " << edgeDistError*edgeDistWeight
            << " Cut " << edgeCut * edgeCutWeight
            << " Match " << edgeMatch * edgeMatchWeight
                 << endl << "\t\t"
                 << " vertexScore: " << vertexScore*vertexWeight << endl << "\t\t"
            << " stdx " << fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight
            << " stdy " << fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight
            << " stdI " << fabs(sd1->gaussians[u].di-sd2->gaussians[v].di) * stdIWeight
            << " meanI " <<    fabs(sd1->gaussians[u].intensity-sd2->gaussians[v].intensity) * meanIWeight
            << " ang " << fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight
                 << endl;

            if(bestScore > score)
            {
                secondBestScore = bestScore;
                bestScore = score;
                vertexIDMatch = v;
                angCorrection = edgeAngError;
            }else if(secondBestScore > score)
            {
                secondBestScore = score;
            }


        }

        cout << u << " -> " << bestScore << " , " << secondBestScore << endl;


        if(bestScore < FLT_MAX)
        {
            if(secondBestScore < FLT_MAX &&
               secondBestScore - bestScore < SIFTCutValue)
                continue;

            if(invScore[vertexIDMatch].first.first > bestScore)
            {
                invScore[vertexIDMatch].second = invScore[vertexIDMatch].first.first;
                invScore[vertexIDMatch].first.first = bestScore;
                invScore[vertexIDMatch].first.second = u;
            }else if(invScore[vertexIDMatch].second > bestScore )
            {
                invScore[vertexIDMatch].second = bestScore;
            }

//            vertexMatch.push_back(pair<unsigned , unsigned>(u,vertexIDMatch));
//            cout << "Match " << u << " com " << vertexIDMatch
//                 << " score " << bestScore << " second score " << secondBestScore << endl;
        }
    }

    for(unsigned i =0  ; i < g2.size() ; i++)
    {
        if(invScore[i].first.first < FLT_MAX)
        {
           if(invScore[i].second - invScore[i].first.first < SIFTCutValue)
            continue;

            vertexMatch.push_back(MatchInfo(invScore[i].first.second,i));
            cout << "Match " << invScore[i].first.second << " com " << i
                 << " score " << invScore[i].first.first << " second score " << invScore[i].second << endl;
        }
    }
}

void GraphMatcher::matche2(SonarDescritor *sd1, SonarDescritor *sd2,
                           vector<MatchInfo> &vertexMatch)
{
/*
    vector< vector<GraphLink*> > &g1 = sd1->graph , &g2 = sd2->graph;
    vector<VertexMatch*> mg1[g1.size()], mg2[g2.size()];


    float score, angCorrection;
    int vertexIDMatch;

    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1.size() <= 1) continue;

        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2.size() <= 1) continue;

            score = compVertex(g1[u],g2[v],&angCorrection);

            if(score < FLT_MAX)
            {
                mg1[u].push_back(new VertexMatch(score,angCorrection,v));
                mg2[v].push_back(new VertexMatch(score,angCorrection,u));
            }
        }

        sort(mg1[u].begin(), mg1[u].end(), compVertexMatch);
    }

    for(unsigned v = 0 ; v < g2.size() ; v++)
    {
        sort(mg2[v].begin(),mg2[v].end(), compVertexMatch);
    }

//    if(vertexIDMatch >=0)
//    {
//        vertexMatch.push_back(MatchInfo(u,vertexIDMatch));
//        cout << "Match " << u << " com " << vertexIDMatch << " score " << bestScore
//             << " AngCorrection " << angCorrection
//             << " translacao = " << sd1->gaussians[u].x - sd2->gaussians[vertexIDMatch].x
//             << " , " << sd1->gaussians[u].y - sd2->gaussians[vertexIDMatch].y << endl;
//    }
*/
}

void GraphMatcher::drawMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                             vector<MatchInfo> &vertexMatch,
                             Mat colorImg)
{
    Drawing::drawDescriptorColor(colorImg,sd1,Scalar(0,255,0),Scalar(255,255,255),false,false,false,true);
    Drawing::drawDescriptorColor(colorImg,sd2, Scalar(255,0,0),Scalar(255,0,255),false,false,false,true);

    for(unsigned i = 0 ; i < vertexMatch.size(); i++)
    {
        unsigned u = vertexMatch[i].uID , v = vertexMatch[i].vID;
        line(colorImg,
             Point2f(sd1->gaussians[u].x, sd1->gaussians[u].y),
             Point2f(sd2->gaussians[v].x, sd2->gaussians[v].y),
             Scalar(0,0,255),2);
    }
}

void GraphMatcher::drawMatchOnImgs(Mat &result,const Size2i &rSZ,
                                   SonarDescritor *sd1, Mat &img1,
                                   SonarDescritor *sd2, Mat &img2,
                                   vector<MatchInfo> &vertexMatch)
{
    result = //Mat::zeros(rSZ.height, rSZ.width,CV_8UC3);
             Mat(rSZ.height, rSZ.width,CV_8UC3);

    Size2i imSZ(rSZ.width/2,rSZ.height);

    Mat cimg1, cimg2;

    cvtColor(img1,cimg1,CV_GRAY2BGR);
    cvtColor(img2,cimg2,CV_GRAY2BGR);

    Drawing::drawDescriptorColor(cimg1,sd1,Scalar(0,255,0),Scalar(255,255,255),false,false,false,true);
    Drawing::drawDescriptorColor(cimg2,sd2, Scalar(255,0,0),Scalar(255,0,255),false,false,false,true);

    resize(cimg1,cimg1,imSZ);
    resize(cimg2,cimg2,imSZ);

    cimg1.copyTo(result(Rect(0         , 0 , imSZ.width , imSZ.height)));
    cimg2.copyTo(result(Rect(imSZ.width, 0 , imSZ.width , imSZ.height)));

    float e1x = (float)imSZ.width/ (float)img1.cols, e1y = (float)imSZ.height / (float)img1.rows,
          d1x = 0 , d1y = 0,
          e2x = (float)imSZ.width/ (float)img2.cols, e2y = (float)imSZ.height / (float)img2.rows,
          d2x = imSZ.width, d2y = 0;

    for(unsigned i = 0 ; i < vertexMatch.size(); i++)
    {
        unsigned u = vertexMatch[i].uID , v = vertexMatch[i].vID;
        line(result,
             Point2f(d1x + e1x*sd1->gaussians[u].x, d1y + e1y*sd1->gaussians[u].y),
             Point2f(d2x + e2x*sd2->gaussians[v].x, d2y + e2y*sd2->gaussians[v].y),
             Scalar(0,0,255),2);
    }

}

void GraphMatcher::drawVertexMatch(Mat colorImg, unsigned idU, const Gaussian& vu , const vector<GraphLink *> &eu, unsigned idV, const Gaussian &vv, const vector<GraphLink *> &ev, const Rect &drawingArea)
{
    float ux = drawingArea.x + drawingArea.width/4.f,
          uy = drawingArea.y + drawingArea.height/2.f,
          vx = ux*3.f,
          vy = uy,
          mr = min(drawingArea.width,drawingArea.height)/2.f;


    char tempStr[40];

    // Find longest edge to normalization after
    unsigned maxECount = max(eu.size(),ev.size()),
             maxEdist = 0.f;
    for(unsigned i =0; i < maxECount ; i++)
    {
        if(i < eu.size() && maxEdist < eu[i]->p)
            maxEdist = eu[i]->p;
        if(i < ev.size() && maxEdist < ev[i]->p)
            maxEdist = ev[i]->p;
    }


    // Draw elipse u
    ellipse(colorImg,
            Point2f(ux, uy),
            Size2f(vu.dx, vu.dy),
            vu.ang,
            0.0 , 360.0,
            Scalar(255,0,0),
            2 , 8 , 0);

    sprintf(tempStr, "ID %d", idU);
    putText(colorImg,tempStr,
           Point2f(ux+20, uy+20),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(0,255,255),2);


    // Draw edges u
    for(unsigned i =0; i < eu.size() ; i++)
    {
        float r = mr*(eu[i]->p/maxEdist),
              ang = eu[i]->ang*M_PI/180.f;

        line(colorImg,Point2f(ux, uy),
             Point2f(ux + r*sin(ang),
                     uy - r*cos(ang)),
             Scalar(255,0,0),2);

        // Draw Edge's angle
        sprintf(tempStr,"p%.2f ; a%.2f ; i%2.f", eu[i]->p , eu[i]->ang,eu[i]->invAngle);

        int baseLine = 0;
        Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX, 0.4,1,&baseLine);

        putText(colorImg, tempStr,
                Point2f(ux + r*sin(ang)-textSize.width/2,
                        uy - r*cos(ang)),
                FONT_HERSHEY_COMPLEX, 0.4,
                Scalar(0,255,255), 1, 8);
    }

    // Draw elipse v
    ellipse(colorImg,
            Point2f(vx, vy),
            Size2f(vv.dx, vv.dy),
            vv.ang,
            0.0 , 360.0,
            Scalar(0,0,255),
            2 , 8 , 0);

    sprintf(tempStr, "ID %d", idV);
    putText(colorImg,tempStr,
           Point2f(vx+20, vy+20),
           FONT_HERSHEY_COMPLEX,0.5,
            Scalar(255,255,0),2);

    // Draw edges v
    for(unsigned i =0; i < ev.size() ; i++)
    {
        float r = mr*(ev[i]->p/maxEdist),
              ang = ev[i]->ang*M_PI/180.f;

        line(colorImg,Point2f(vx, vy),
             Point2f(vx + r*sin(ang),
                     vy - r*cos(ang)),
             Scalar(0,0,255),2);

        // Draw Edge's angle
        sprintf(tempStr,"p%.2f ; a%.2f ; i%2.f",ev[i]->p, ev[i]->ang,ev[i]->invAngle);

        int baseLine = 0;
        Size textSize = getTextSize(tempStr,FONT_HERSHEY_COMPLEX, 0.4,1,&baseLine);

        putText(colorImg, tempStr,
                Point2f(vx + r*sin(ang)-textSize.width/2,
                        vy - r*cos(ang)),
                FONT_HERSHEY_COMPLEX, 0.4,
                Scalar(255,255,0), 1, 8);

    }
}

