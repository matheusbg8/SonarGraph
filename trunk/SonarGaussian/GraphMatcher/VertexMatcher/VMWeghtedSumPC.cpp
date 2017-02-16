#include "VMWeghtedSumPC.h"

VMWeghtedSumPC::VMWeghtedSumPC():
    rhoMax(5.f),thetaMax(15.f),rhoW(3.0f),thetaW(25.0f),
    sigmaHitW(-15.f),sigmaMissW(0.f),
    stdXWeight(1.f),stdYWeight(1.f),stdIWeight(0.f),
    meanIWeight(1.f),angWeight(1.f)
{
}

bool VMWeghtedSumPC::load(ConfigLoader &config)
{
    bool gotSomeConfig=false;
    float fv;

    if(config.getFloat("VMWeghtedSumPC","rhoMax",&fv))
    {
        rhoMax= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMWeghtedSumPC","thetaMax",&fv))
    {
        thetaMax= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMWeghtedSumPC","rhoW",&fv))
    {
        rhoW= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMWeghtedSumPC","thetaW",&fv))
    {
        thetaW= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMWeghtedSumPC","sigmaHitW",&fv))
    {
        sigmaHitW= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMWeghtedSumPC","sigmaMissW",&fv))
    {
        sigmaMissW= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMWeghtedSumPC","angWeight",&fv))
    {
        angWeight= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMWeghtedSumPC","meanIWeight",&fv))
    {
        meanIWeight= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMWeghtedSumPC","stdXWeight",&fv))
    {
        stdXWeight= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMWeghtedSumPC","stdYWeight",&fv))
    {
        stdYWeight= fv;
        gotSomeConfig=true;
    }

    if(config.getFloat("VMWeghtedSumPC","stdIWeight",&fv))
    {
        stdIWeight= fv;
        gotSomeConfig=true;
    }

    return gotSomeConfig;
}

float VMWeghtedSumPC::vertexMatch(Gaussian &gu, Gaussian &gv,
                                  vector<GraphLink *> &eu, vector<GraphLink *> &ev,
                                  unsigned *matchEdges)
{

    // ** We consider that edges are in increasing order by ang
    sort(eu.begin(),eu.end(),compRelativeAng);
    sort(ev.begin(),ev.end(),compRelativeAng);

    unsigned i=0, j=0,
            edgeMatchCount =0,
            edgeCutCount=0;

    float angScore=0.f, distScore=0.f;

    while( i < eu.size() && j < ev.size())
    {
        float diffDistance = fabs(eu[i]->p - ev[j]->p),
              diffAng = fabs(eu[i]->rAng - ev[j]->rAng);

        if(diffDistance < rhoMax && diffAng < thetaMax)
        {
            angScore += diffAng;
            distScore += diffDistance;
            edgeMatchCount++;
            i++; j++;
        }else
        {
            edgeCutCount++;
            if(eu[i]->ang < ev[j]->ang)
                i++;
            else j++;
        }
    }
    if(edgeMatchCount > 0)
    {
        angScore/= edgeMatchCount;
        distScore/= edgeMatchCount;
    }

    if(matchEdges != 0x0)
        *matchEdges = edgeMatchCount;

    return angScore*thetaW + distScore*rhoW +
           edgeMatchCount*sigmaHitW + edgeCutCount*sigmaMissW +
           fabs(gu.dx-gv.dx) * stdXWeight +
           fabs(gu.dy-gv.dy) * stdYWeight +
           fabs(gu.di-gv.di) * stdIWeight +
           fabs(gu.intensity-gv.intensity) * meanIWeight +
           fabs(gu.ang-gv.ang) * angWeight;
}

/**
 * @brief
 * ///@todo - Implement VMWeghtedSumPC::vertexMatch for debug mode.
 * @param gu
 * @param gv
 * @param u
 * @param v
 * @param matchEdges
 * @return float
 */
float VMWeghtedSumPC::vertexMatch(Gaussian &gu, Gaussian &gv,
                                  vector<GraphLink *> &u, vector<GraphLink *> &v,
                                  vector<MatchInfoWeighted> &matchEdges)
{
    // ** We consider that edges are in increasing order by ang

//    sort(eu.begin(),eu.end(),compRelativeAng);
//    sort(ev.begin(),ev.end(),compRelativeAng);

//    unsigned i=0, j=0;
//    (*edgeMatch)=0; (*edgeCut)=0;
//    (*angScore)=0.f; (*distScore)=0.f;

//    while( i < eu.size() && j < ev.size())
//    {
//        float diffDistance = fabs(eu[i]->p - ev[j]->p),
//              diffAng = fabs(eu[i]->rAng - ev[j]->rAng);

//        if(diffDistance < rhoMax && diffAng < thetaMax)
//        {
//            (*angScore)+= diffAng;
//            (*distScore) += diffDistance;
//            (*edgeMatch)++;
//            i++; j++;
//        }else
//        {
//            (*edgeCut)++;
//            if(eu[i]->ang < ev[j]->ang)
//                i++;
//            else j++;
//        }
//    }
//    if((*edgeMatch) > 0)
//    {
//        (*angScore)/= (*edgeMatch);
//        (*distScore)/= (*edgeMatch);
//    }
}
