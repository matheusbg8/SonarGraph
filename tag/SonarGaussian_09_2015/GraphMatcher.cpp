#include "GraphMatcher.h"
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <stack>

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

void GraphMatcher::staticCopyVertex(const vector<GraphLink *> &source, vector<GraphLink> &dest)
{
    dest.resize(source.size(), 0x0);
    for(unsigned i =0; i < source.size(); i++)
        dest[i] = (*(source[i]));
}

void GraphMatcher::dinamicCopyVertex(const vector<GraphLink *> &source, vector<GraphLink*> &dest)
{
    dest.resize(source.size(), 0x0);
    for(unsigned i =0; i < source.size(); i++)
        dest[i] = new GraphLink(*(source[i]));
}

void GraphMatcher::freeVertex(vector<GraphLink *> &vertex)
{
    for(unsigned i =0; i < vertex.size(); i++)
        delete vertex[i];
    vertex.clear();
}

void GraphMatcher::copyVertexList(const vector<GraphLink *> &source, list<GraphLink> &dest)
{
    for(unsigned i = 0 ; i < source.size(); i++)
        dest.push_back(*(source[i]));
}

void GraphMatcher::computeEdgeError(vector<GraphLink *> &u, vector<GraphLink *> &v,
                                    float *angScore, float *distScore,
                                    unsigned *edgeMatch, unsigned *edgeCut)
{
    // ** We consider that edges are in an increasing order

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

void GraphMatcher::brutalFORCExplore(vector<GraphLink *> &u,
                                     vector< vector< vector<PFPUU> > > &explore)
{
//    vector< // Group Size
//            vector< // Group
//                vector<PFPUU> // Elements
//                  >
//          > explore;

    explore.resize(u.size()+1,vector< vector<PFPUU> >());


    explore[u.size()].push_back( vector<PFPUU> ());
    for(unsigned i = 0; i < u.size()-1; i++)
    {
        explore[u.size()][0].push_back(PFPUU(u[i]->invAngle,PUU(i,i+1)));
    }
    explore[u.size()][0].push_back(PFPUU(u[u.size()-1]->invAngle,PUU(u.size()-1,0)));

    stack<
         pair<unsigned,// setSize
            pair<unsigned, // setID
                 unsigned  // postion of last swap
                >
             >
         > q; // Recursive Stack!

    q.push(PUPUU(u.size(),PUU(0,0)));
    unsigned setSize,nextSize, setID,lastSwapPostion;

    while(!q.empty())
    {
        setSize = q.top().first;
        setID = q.top().second.first;
        lastSwapPostion = q.top().second.second;
        q.pop();

        nextSize = setSize-1;

        vector<PFPUU> &exp = explore[setSize][setID];
        unsigned iSwap, jSwap;

        // Changes aways start at beginner (iSwap=0)
        for(lastSwapPostion, iSwap =0; lastSwapPostion < setSize; lastSwapPostion++, iSwap++)
        {
            // Create a new set
            unsigned newSetID = explore[nextSize].size();
            explore[nextSize].push_back( vector<PFPUU> ());
            vector<PFPUU> &newExp = explore[nextSize][newSetID];
            newExp.reserve(nextSize);

            jSwap = (iSwap+1)%setSize;

            // Blend two elements (iSwap with jSwap)
           newExp.push_back(PFPUU(
                                 exp[iSwap].first + exp[jSwap].first,
                                 PUU(exp[iSwap].second.first, exp[jSwap].second.second)
                                  )
                             );

            unsigned i = (iSwap+2)%setSize,
                     m = setSize-3;

            // Copy remainded elements
            while(m < 999999)
            {
                newExp.push_back(exp[i]);
                i=(i+1)%setSize; // Circular increment
                m--;
            }

            // Work with new set created
            if(nextSize>2 && lastSwapPostion < nextSize)
            {
                q.push(PUPUU(nextSize,PUU(newSetID,lastSwapPostion)));
            }
        }
    }

#ifdef DEBUG_GRAPHMACHER
    for(unsigned setSize = u.size(); setSize > 1;setSize--)
    {
        for(unsigned setID = 0 ; setID < explore[setSize].size(); setID++)
        {
            for(unsigned k = 0 ; k < explore[setSize][setID].size() ; k++)
            {
                cout << "(" << explore[setSize][setID][k].first
                     << " , " << explore[setSize][setID][k].second.first
                     << " , " << explore[setSize][setID][k].second.second
                     << " ) ";
            }
            cout << endl;
        }
    }
#endif
}

void GraphMatcher::brutalFORCEEdgeConfigCompare(vector<GraphLink *> &u, vector<GraphMatcher::PFPUU> &uExp,
                                                vector<GraphLink *> &v, vector<GraphMatcher::PFPUU> &vExp)
{
//    if(uExp.size() > 0 && uExp.size() != vExp.size())
//        return;

//    unsigned nEdges = uExp.size(),
//             nComb = nEdges-1;

//    float bestScore=999999.f,
//          angDiff, distDiff;

//    int bestComb = -1;

//    for(unsigned comb = 0 ; comb < nComb; comb++)
//    {
//        float angMeanDiff = 0.f,
//              distMeanDiff = 0.f;

//        unsigned i = 0;
//        while(i < nEdges)
//        {
//            angDiff = fabs(uExp[i].first - vExp[(i+comb)%nEdges].first);
//            if(angDiff > angThreshold)
//                break;

//            distDiff = fabs(uExp[i].second.first - vExp[(i+comb)%nEdges].second.first);
//            if(distDiff > distThreshold)
//                break;
//            // The second edge will be tested ont the next iteration

//            angMeanDiff += angDiff;
//            distMeanDiff += distDiff;

//            i++;
//        }

//        if(i < nEdges)
//            continue;

//        // OK, we find a match
//        // now, we will evaluate this match

//        angMeanDiff /= nEdges;
//        distMeanDiff /= nEdges;

//        if(angMeanDiff)
//    }
}

void GraphMatcher::brutalFORCECompare(vector<GraphLink *> &u, vector<vector<vector<GraphMatcher::PFPUU> > > &uExp,
                                      vector<GraphLink *> &v, vector<vector<vector<GraphMatcher::PFPUU> > > &vExp)
{
    unsigned uI=0, vI=0;

}

void GraphMatcher::brutalFORCE(vector<GraphLink *> &u,
                               vector<GraphLink *> &v)
{
    vector< vector<
            pair<
                float,
                PUU
            > >
            >
            combU, combV;



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

    distThreshold = 8.f;
    angThreshold = 18.f;

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

    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1[u].size() <= 1) continue;

        bestScore = FLT_MAX;
        vertexIDMatch=-1;

        for(unsigned v = 0 ; v < g2.size(); v++)
        {
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
                    fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight +
                    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight +
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

void GraphMatcher::matcheSIFTCut(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch)
{
    vector< vector<GraphLink*> > &g1 = sd1->graph,
                                 &g2 = sd2->graph;

    typedef pair<float,int> PFI;
    typedef pair< PFI , float> PFIF;

    vector<PFIF> invScore(g2.size(), PFIF(PFI(FLT_MAX,-1) ,FLT_MAX));

    float score, bestScore = FLT_MAX, secondBestScore,
          edgeScore=0.f, vertexScore,
          angCorrection=0.f,
          edgeDistError=0.f, edgeAngError=0.f;

    int vertexIDMatch=-1;

    unsigned edgeCut=0, edgeMatch=0;

    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1[u].size() < 6) continue;
        bestScore = secondBestScore = FLT_MAX;
        vertexIDMatch=-1;

        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2[v].size() < 6) continue;
            // Compute Edge Error
            computeEdgeError(g1[u], g2[v],
                             &edgeAngError,&edgeDistError,
                             &edgeMatch, &edgeCut);

            if(edgeMatch < 6) continue;

            edgeScore = edgeAngError*edgeAngWeight
                        + edgeDistError*edgeDistWeight
//                        + edgeCut * edgeCutWeight
//                      + edgeMatch * edgeMatchWeight
                    ;

            //Compute the vertex Error
            vertexScore = fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight +
                    fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight +
                    fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight +
                    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight +
                    fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight;
                    ;

            score = edgeScore*edgeWeight + vertexScore*vertexWeight;
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

            vertexMatch.push_back(pair<unsigned , unsigned>(invScore[i].first.second,i));
//            cout << "Match " << invScore[i].first.second << " com " << i
//                 << " score " << invScore[i].first.first << " second score " << invScore[i].second << endl;
        }
    }
}

void GraphMatcher::matcheSIFTCutDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                                      vector<pair<unsigned, unsigned> > &vertexMatch,
                                      vector<SpatialMatchInfo> &matchInfo)
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
        if(g1[u].size() < 6) continue;

        bestScore = secondBestScore = FLT_MAX;
        vertexIDMatch=-1;

        // For each gaussian v in img2
        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2[v].size() < 6) continue;

            // Compute Edge Error
            computeEdgeError(g1[u], g2[v],
                             &edgeAngError,&edgeDistError,
                             &edgeMatch, &edgeCut);

            if(edgeMatch < 6) continue;

            edgeScore = edgeAngError*edgeAngWeight
                        + edgeDistError*edgeDistWeight
//                        + edgeCut * edgeCutWeight
//                      + edgeMatch * edgeMatchWeight
                    ;

            //Compute the vertex Error
            vertexScore = fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight +
                    fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight +
                    fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight +
                    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight +
                    fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight;
                    ;

            score = edgeScore*edgeWeight + vertexScore*vertexWeight;
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

           matchInfo.push_back(SpatialMatchInfo());
           SpatialMatchInfo &info = matchInfo.back();

           info.bestScore = invScore[i].first.first;
           info.sBestScore = invScore[i].second;

           info.uID = invScore[i].first.second;
           info.vID = i;

           computeEdgeMatch(g1[info.uID], g2[info.vID],
                            info.edgeMatchInfo);

            vertexMatch.push_back(pair<unsigned , unsigned>(invScore[i].first.second,i));

//            cout << "Match " << invScore[i].first.second << " com " << i
//                 << " score " << invScore[i].first.first << " second score " << invScore[i].second << endl;
        }
    }
}

void GraphMatcher::matcheM(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch)
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
                    fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight +
                    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight +
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
            << " stdI " << fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight
            << " meanI " <<    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight
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

            vertexMatch.push_back(pair<unsigned , unsigned>(invScore[i].first.second,i));
            cout << "Match " << invScore[i].first.second << " com " << i
                 << " score " << invScore[i].first.first << " second score " << invScore[i].second << endl;
        }
    }
}

void GraphMatcher::matche2(SonarDescritor *sd1, SonarDescritor *sd2, vector< pair<unsigned, unsigned> > &vertexMatch)
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
//        vertexMatch.push_back(pair<unsigned,unsigned>(u,vertexIDMatch));
//        cout << "Match " << u << " com " << vertexIDMatch << " score " << bestScore
//             << " AngCorrection " << angCorrection
//             << " translacao = " << sd1->gaussians[u].x - sd2->gaussians[vertexIDMatch].x
//             << " , " << sd1->gaussians[u].y - sd2->gaussians[vertexIDMatch].y << endl;
//    }
*/
}

void GraphMatcher::drawMatch(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch, Mat colorImg)
{
    SonarDescritor::drawDescriptorColor(colorImg,sd1,Scalar(0,255,0),Scalar(255,255,255),false,false,false,true);
    SonarDescritor::drawDescriptorColor(colorImg,sd2, Scalar(255,0,0),Scalar(255,0,255),false,false,false,true);

    for(unsigned i = 0 ; i < vertexMatch.size(); i++)
    {
        unsigned u = vertexMatch[i].first , v = vertexMatch[i].second;
        line(colorImg,
             Point2f(sd1->gaussians[u].x, sd1->gaussians[u].y),
             Point2f(sd2->gaussians[v].x, sd2->gaussians[v].y),
             Scalar(0,0,255),2);
    }
}

void GraphMatcher::drawMatchOnImgs(Mat &result,const Size2i &rSZ, SonarDescritor *sd1, Mat &img1, SonarDescritor *sd2, Mat &img2, vector<pair<unsigned, unsigned> > &vertexMatch)
{
    result = //Mat::zeros(rSZ.height, rSZ.width,CV_8UC3);
             Mat(rSZ.height, rSZ.width,CV_8UC3);

    Size2i imSZ(rSZ.width/2,rSZ.height);

    Mat cimg1, cimg2;

    cvtColor(img1,cimg1,CV_GRAY2BGR);
    cvtColor(img2,cimg2,CV_GRAY2BGR);

    SonarDescritor::drawDescriptorColor(cimg1,sd1,Scalar(0,255,0),Scalar(255,255,255),false,false,false,true);
    SonarDescritor::drawDescriptorColor(cimg2,sd2, Scalar(255,0,0),Scalar(255,0,255),false,false,false,true);

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
        unsigned u = vertexMatch[i].first , v = vertexMatch[i].second;
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
    float drawAng = vu.ang;
    if(vu.dx > vu.dy)
    {
        if(drawAng<0.f)
            drawAng =   90.f + vu.ang;
        else drawAng = -90.f + vu.ang;
    }

    ellipse(colorImg,
            Point2f(ux, uy),
            Size2f(vu.dx, vu.dy),
            drawAng,
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
    drawAng = vv.ang;
    if(vv.dx > vv.dy)
    {
        if(drawAng<0.f)
            drawAng =   90.f + vv.ang;
        else drawAng = -90.f + vv.ang;
    }

    ellipse(colorImg,
            Point2f(vx, vy),
            Size2f(vv.dx, vv.dy),
            drawAng,
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
