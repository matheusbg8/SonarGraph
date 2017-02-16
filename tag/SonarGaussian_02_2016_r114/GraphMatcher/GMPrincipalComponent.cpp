#include "GMPrincipalComponent.h"

GMPrincipalComponent::GMPrincipalComponent()
{
}

void GMPrincipalComponent::matche(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch)
{
//    // This is our graph representation, a vector of vertex
//    vector< vector<GraphLink*> > &g1 = sd1->graph,
//                                 &g2 = sd2->graph;

//    // This is our score match dta struct
//    typedef pair<float,int> PFI;
//    typedef pair< PFI , float> PFIF;

//    vector<PFIF> invScore(g2.size(), PFIF(PFI(FLT_MAX,-1) ,FLT_MAX));

//    float score, bestScore = FLT_MAX, secondBestScore,
//          edgeScore=0.f, vertexScore,
//          angCorrection=0.f,
//          edgeDistError=0.f, edgeAngError=0.f;

//    int vertexIDMatch=-1;

//    unsigned edgeCut=0, edgeMatch=0;

//    for(unsigned u = 0 ; u < g1.size() ; u++)
//    {
//        if(g1[u].size() < minSimilarEdgeToMatch) continue;
//        bestScore = secondBestScore = FLT_MAX;
//        vertexIDMatch=-1;

//        for(unsigned v = 0 ; v < g2.size(); v++)
//        {
//            if(g2[v].size() < minSimilarEdgeToMatch) continue;

//            // Compute Edge Error
//            computeEdgeError(g1[u], g2[v],
//                             &edgeAngError,&edgeDistError,
//                             &edgeMatch, &edgeCut);

//            if(edgeMatch < minSimilarEdgeToMatch) continue;

//            edgeScore = edgeAngError*edgeAngWeight
//                        + edgeDistError*edgeDistWeight
////                        + edgeCut * edgeCutWeight
////                      + edgeMatch * edgeMatchWeight
//                    ;

//            //Compute the vertex Error
//            vertexScore = fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight +
//                    fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight +
//                    fabs(sd1->gaussians[u].di-sd2->gaussians[v].di) * stdIWeight +
//                    fabs(sd1->gaussians[u].intensity-sd2->gaussians[v].intensity) * meanIWeight +
//                    fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight;
//                    ;

//            score = edgeScore*edgeWeight + vertexScore*vertexWeight;
///*
//            cout << '\t' << u << " -> " << v << " : " << score
//                 << endl << "\t\t"
//                 <<  " edgeScore: " << edgeScore*edgeWeight << endl << "\t\t"
//            << " Ang " << edgeAngError*edgeAngWeight
//            << " Dist " << edgeDistError*edgeDistWeight
//            << " Cut " << edgeCut * edgeCutWeight
//            << " Match " << edgeMatch * edgeMatchWeight
//                 << endl << "\t\t"
//                 << " vertexScore: " << vertexScore*vertexWeight << endl << "\t\t"
//            << " stdx " << fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight
//            << " stdy " << fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight
//            << " stdI " << fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight
//            << " meanI " <<    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight
//            << " ang " << fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight
//                 << endl;
//*/
//            if(bestScore > score)
//            {
//                secondBestScore = bestScore;
//                bestScore = score;
//                vertexIDMatch = v;
//                angCorrection = edgeAngError;
//            }else if(secondBestScore > score)
//            {
//                secondBestScore = score;
//            }

//            if(drawVertexMatching)
//            { /*
//                int w=1800 , h=900;
//                Mat colorImg = Mat::zeros(Size(w,h),CV_8UC3);
//                drawVertexMatch(colorImg,u,sd1->gaussians[u],g1[u],v,sd2->gaussians[v],g2[v],Rect(0,0,w,h/2));
//                drawVertexMatch(colorImg,u,sd1->gaussians[u],cu,v,sd2->gaussians[v],cv,Rect(0,h/2,w,h/2));
//                imshow("VertexMatch",colorImg);
//                waitKey();
//              */
//            }
//        }

////        cout << u << " -> " << bestScore << " , " << secondBestScore << endl;


//        if(bestScore < FLT_MAX)
//        {
//            if(secondBestScore < FLT_MAX &&
//               secondBestScore - bestScore < SIFTCutValue)
//                continue;

//            if(invScore[vertexIDMatch].first.first > bestScore)
//            {
//                invScore[vertexIDMatch].second = invScore[vertexIDMatch].first.first;
//                invScore[vertexIDMatch].first.first = bestScore;
//                invScore[vertexIDMatch].first.second = u;
//            }else if(invScore[vertexIDMatch].second > bestScore )
//            {
//                invScore[vertexIDMatch].second = bestScore;
//            }

////            vertexMatch.push_back(pair<unsigned , unsigned>(u,vertexIDMatch));
////            cout << "Match " << u << " com " << vertexIDMatch
////                 << " score " << bestScore << " second score " << secondBestScore << endl;
//        }
//    }

//    for(unsigned i =0  ; i < g2.size() ; i++)
//    {
//        if(invScore[i].first.first < FLT_MAX)
//        {
//           if(invScore[i].second - invScore[i].first.first < SIFTCutValue)
//            continue;

//            vertexMatch.push_back(pair<unsigned , unsigned>(invScore[i].first.second,i));
////            cout << "Match " << invScore[i].first.second << " com " << i
////                 << " score " << invScore[i].first.first << " second score " << invScore[i].second << endl;
//        }
//    }
}

void GMPrincipalComponent::matcheDebug(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch, vector<MatchInfoExtended> &matchInfo)
{

}
