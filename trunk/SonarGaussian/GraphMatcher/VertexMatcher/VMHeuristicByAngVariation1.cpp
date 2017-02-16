#include "VMHeuristicByAngVariation1.h"

VMHeuristicByAngVariation1::VMHeuristicByAngVariation1()
{
}

bool VMHeuristicByAngVariation1::load(ConfigLoader &config)
{

}

float VMHeuristicByAngVariation1::vertexMatch(Gaussian &gu, Gaussian &gv,
                                              vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                                              unsigned *matchEdges)
{
    //void GraphMatcher::findPairOfVertexByDistance(vector<GraphLink *> &u, vector<GraphLink *> &v, float *edgeAngError, float *edgeDistError, float *angCorrectionMean,unsigned *edgeMatch, unsigned *edgeCut)

        /*
            Problema desta solucao:
                As vezes a aresta que melhor encaixa é menor
            do que uma aresta que tambem encaixa.
                E se tiver duas aresta de tamanho igual mas
            angulos diferentes ? E só uma delas encaixa e a outra
            não está presente no proximo grafo.
        */

            // Sort edges by weight increasing order (distances vertex to vertex)
    //        sort(u.begin(),u.end(), compDist);
    //        sort(v.begin(),v.end(), compDist);

    //        int i , j;
    //        float distError, angError, ACangCorrectionMean=0.f;

    //        *edgeDistError=0.f;
    //        *edgeAngError=0.f;
    //        *edgeCut = *edgeMatch =0;

    //        // Compute edges match by wight (vertex distances)
    //        j = 0;
    //        i = 0;
    //        while(i < u.size() && j < v.size())
    //        {
    //            distError = fabs(u[i]->p - v[j]->p);

    //            if(distError > distThreshold)
    //            {   // cut no match edge

    //                // cut the edge with lowest weight
    //                if(v[i]->p < u[j]->p)
    //                {
    //                    (*edgeDistError)+= v[i]->p;
    //                    (*edgeCut)++;
    //                    v.erase(v.begin()+j);
    //                }else
    //                {
    //                    (*edgeCut)++;
    //                    u.erase(u.begin()+i);
    //                }
    //            }else
    //            {
    //                (*edgeDistError) += distError;
    //                ACangCorrectionMean+= u[i]->ang - v[j]->ang;
    //                i++;
    //                j++;
    //            }
    //        }

    //        while(i < u.size())
    //        {
    //            (*edgeCut)++;
    //            u.erase(u.begin()+i);
    //        }

    //        while(j < v.size())
    //        {
    //            (*edgeCut)++;
    //            v.erase(v.begin()+j);
    //        }

    //        *angCorrectionMean = ACangCorrectionMean/u.size();

    //        if(u.size() <= 1 || v.size() <= 1)
    //        {
    //            // No match found
    //            return;
    //        }

    //        // Compute vertex match by invAng (Edges distances)
    //        GraphLink::computeInvAng(v);// Change the edges order, it's important
    //        GraphLink::computeInvAng(u);// Change the edges order

    //        for(i = 0; i < v.size(); i++)
    //        {
    //            angError = fabs(v[i]->invAngle - u[i]->invAngle);
    //            distError = fabs(v[i]->p - u[i]->p);
    //            if(angError <= angThreshold &&
    //               distError <= distThreshold )
    //            {   // Match found
    //                (*edgeMatch)++;
    //                *edgeAngError += angError;
    //            }else
    //            {
    //                (*edgeCut)+= v.size();
    //                (*edgeMatch) =0;
    //                return;
    //            }
    //        }
    //        *edgeAngError/= *edgeMatch;
}

float VMHeuristicByAngVariation1::vertexMatch(Gaussian &gu, Gaussian &gv,
                                              vector<GraphLink *> &u, vector<GraphLink *> &v,
                                              vector<MatchInfoWeighted> &matchEdges)
{

}
