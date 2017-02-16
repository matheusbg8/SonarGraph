#include "GMFVertexByVertex.h"

GMFVertexByVertex::GMFVertexByVertex():
    meaningfulness(5.f),minSimilarEdgeToMatch(4u)
{

}

bool GMFVertexByVertex::load(ConfigLoader &config)
{
    float fv;
    int iv;
    bool gotSomeConfig=false;

    if(config.getFloat("GMFVertexByVertex","meaningfulness",&fv))
    {
        meaningfulness = fv;
        gotSomeConfig = true;
    }

    if(config.getInt("GMFVertexByVertex","minSimilarEdgeToMatch",&iv))
    {
        minSimilarEdgeToMatch = iv;
        gotSomeConfig = true;
    }

    return gotSomeConfig;

}

void GMFVertexByVertex::findMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                                  vector<MatchInfo> &vertexMatch)
{
    vector< vector<GraphLink*> > &g1 = sd1->graph,
                                 &g2 = sd2->graph;

    typedef pair<float,int> PFI;
    typedef pair< PFI , float> PFIF;

    vector<PFIF> invScore(g2.size(), PFIF(PFI(FLT_MAX,-1) ,FLT_MAX));
    // invScore[ destID ] ( ( bestScore , srcIDBest) , SecondBestScore)

    float score,
          bestScore = FLT_MAX,
          secondBestScore = FLT_MAX;

    int vertexIDMatch=-1;
    unsigned edgeMatch=0;

    // For each gaussian u
    for(unsigned u = 0 ; u < g1.size() ; u++)
    {
        if(g1[u].size() < minSimilarEdgeToMatch) continue;

        bestScore = secondBestScore = FLT_MAX;
        vertexIDMatch=-1;

        // For each gaussian v
        for(unsigned v = 0 ; v < g2.size(); v++)
        {
            if(g2[v].size() < minSimilarEdgeToMatch) continue;

            score = m_vertexMatcher->vertexMatch(sd1->gaussians[u],sd2->gaussians[v],
                                                 g1[u],g2[v],&edgeMatch);

            if(edgeMatch < minSimilarEdgeToMatch) continue;

            if(bestScore > score)
            {
                secondBestScore = bestScore;
                bestScore = score;
                vertexIDMatch = v;
            }else if(secondBestScore > score)
            {
                secondBestScore = score;
            }
        }

//        cout << u << " -> " << bestScore << " , " << secondBestScore << endl;

        // If it was a match
        if(bestScore < FLT_MAX)
        {
            // If it was two matchs
            if(secondBestScore < FLT_MAX &&
               secondBestScore - bestScore < meaningfulness)
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
           if(invScore[i].second - invScore[i].first.first < meaningfulness)
            continue;

            vertexMatch.push_back(MatchInfo(invScore[i].first.second,i));

//            cout << "Match " << invScore[i].first.second << " com " << i
//                 << " score " << invScore[i].first.first << " second score " << invScore[i].second << endl;
        }
    }
}

void GMFVertexByVertex::findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                                       vector<MatchInfoExtended> &matchInfo)
{
//    vector< vector<GraphLink*> > &g1 = sd1->graph,
//                                 &g2 = sd2->graph;

//    typedef pair<float,int> PFI;
//    typedef pair< PFI , float> PFIF;

//    vector<PFIF> invScore(g2.size(), PFIF(PFI(FLT_MAX,-1) ,FLT_MAX));
//    // invScore[ destID ] ( ( bestScore , srcIDBest) , SecondBestScore)

//    float score, bestScore = FLT_MAX, secondBestScore,
//          edgeScore=0.f, vertexScore,
//          angCorrection=0.f,
//          edgeDistError=0.f, edgeAngError=0.f;

//    int vertexIDMatch=-1;

//    unsigned edgeCut=0, edgeMatch=0;

//    // For each gaussian u in img1
//    for(unsigned u = 0 ; u < g1.size() ; u++)
//    {
//        if(g1[u].size() < minSimilarEdgeToMatch) continue;

//        bestScore = secondBestScore = FLT_MAX;
//        vertexIDMatch=-1;

//        // For each gaussian v in img2
//        for(unsigned v = 0 ; v < g2.size(); v++)
//        {
//            if(g2[v].size() < minSimilarEdgeToMatch) continue;

//            // Compute Edge Error
////            computeEdgeError(g1[u], g2[v],
////                             &edgeAngError,&edgeDistError,
////                             &edgeMatch, &edgeCut);

//            computeEdgeErrorEscaleno(g1[u], g2[v],
//                             &edgeScore,
//                             &edgeMatch, &edgeCut);

//            if(edgeMatch < minSimilarEdgeToMatch) continue;

////            edgeScore = edgeAngError*edgeAngWeight
////                        + edgeDistError*edgeDistWeight
////                        + edgeCut * edgeCutWeight
////                      + edgeMatch * edgeMatchWeight
//                    ;

//            //Compute the vertex Error
////            vertexScore = fabs(sd1->gaussians[u].dx-sd2->gaussians[v].dx) * stdXWeight +
////                    fabs(sd1->gaussians[u].dy-sd2->gaussians[v].dy) * stdYWeight +
////                    fabs(sd1->gaussians[u].dz-sd2->gaussians[v].dz) * stdIWeight +
////                    fabs(sd1->gaussians[u].z-sd2->gaussians[v].z) * meanIWeight +
////                    fabs(sd1->gaussians[u].ang-sd2->gaussians[v].ang) * angWeight;
//                    ;

////            score = edgeScore*edgeWeight + vertexScore*vertexWeight;
//            score = edgeScore;
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
