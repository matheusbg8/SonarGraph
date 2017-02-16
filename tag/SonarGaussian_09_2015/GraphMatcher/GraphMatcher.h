#ifndef GRAPHMATCHER_H
#define GRAPHMATCHER_H

// ===== DEBUG ===========
// #define DEBUG_GRAPHMACHER
// ==== END DEBUG ========

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include <SonarDescritor.h>
#include <vector>
#include <list>
#include <stack>

#include "SonarConfig.h"
#include "SpatialMatchInfo.h"

using namespace std;
using namespace cv;


class GraphMatcher
{
    typedef pair<float,float> PFF;
    typedef pair<unsigned,unsigned> PUU;
    typedef pair<float,PUU> PFPUU;
    typedef pair<unsigned,PUU> PUPUU;
    typedef pair<PFF,PUU> PUUPFF;
private:
    float distThreshold;
    float angThreshold;
    float errorThreshold;

    float stdXWeight;
    float stdYWeight;
    float stdIWeight;
    float meanIWeight;
    float angWeight;

    float edgeAngWeight;
    float edgeDistWeight;
    float edgeCutWeight;
    float edgeMatchWeight;

    float edgeWeight;
    float vertexWeight;

    float SIFTCutValue;

    bool drawVertexMatching;

    unsigned minSimilarEdgeToMatch;

    static void staticCopyVertex(const vector<GraphLink*> &source, vector<GraphLink> &dest);
    static void dinamicCopyVertex(const vector<GraphLink*> &source, vector<GraphLink *> &dest);
    static void freeVertex(vector<GraphLink *> &vertex);
    static void copyVertexList(const vector<GraphLink*> &source, list<GraphLink> &dest);

    void computeEdgeError(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          float *angScore, float *distScore,
                          unsigned *edgeMatch, unsigned *edgeCut);

    void computeEdgeErrorEscaleno(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          float *errorScore, unsigned *edgeMatch,
                          unsigned *edgeCut);

    void computeEdgeMatch(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          vector<PUUPFF> &matchEdges);

    void computeEdgeMatchEscaleno(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          vector<PUUPFF> &matchEdges);

    void computeEdgeErrorGula(vector<GraphLink *> &u, vector<GraphLink *> &v,
                              float *angScore, float *distScore,
                              unsigned *edgeMatch, unsigned *edgeCut);

    bool findPairOfVertex(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          float *invAngError, float *edgeDistError,
                          float *angCorrectionMean, int *edgeCut);

    void findPairOfVertexByDistance(vector<GraphLink *> &u, vector<GraphLink *> &v, float *edgeAngError, float *edgeDistError, float *angCorrectionMean,unsigned *edgeMatch, unsigned *edgeCut);

    void compVertexDist(const vector<GraphLink *> &u, const vector<GraphLink *> &v);

    void buscaMatch(GraphLink *link, vector<GraphLink*> vertex, int star, int end, float ang);

    void findVerticesMatchByAngDiff(vector<GraphLink *> &u, vector<GraphLink *> &v);


    void findBestEdgeMatch(vector<GraphLink *> &u, vector<GraphLink *> &v,
                           unsigned ui, unsigned vi,
                           unsigned &uj, unsigned &vj);

public:
    void brutalFORCExplore(vector<GraphLink *> &u,
                           vector< vector< vector<PFPUU> > > &explore);

    void brutalFORCEEdgeConfigCompare(vector<GraphLink *> &u, vector<PFPUU> &uExp,
                                      vector<GraphLink *> &v, vector<PFPUU> &vExp
                                     );

    void brutalFORCECompare(vector<GraphLink *> &u,vector< vector< vector<PFPUU> > > &uExp,
                            vector<GraphLink *> &v,vector< vector< vector<PFPUU> > > &vExp
                            );

    void brutalFORCE(vector<GraphLink *> &u, vector<GraphLink *> &v);


    void byDistanceCompare(vector<GraphLink *> &u,
                           vector<GraphLink *> &v
                           );

    void byDistanceCompare_(vector<GraphLink *> &u,
                           vector<GraphLink *> &v,
                           vector<float> *edgeMatch
                           );

    void byDistanceCompare_rec(vector<GraphLink *> &u, unsigned uIni,
                           vector<GraphLink *> &v, unsigned vIni,
                           vector<float> *edgeMatch
                           );

public:

    static float diffAngle(vector<GraphLink *> &u,
                           unsigned fromEdgeID, unsigned toEdgeID);

    inline static float scaleneError(float degreeAngDiff, float length1 , float length2);


    GraphMatcher(const SonarConfig& config = SonarConfig());

    void matche(SonarDescritor *sd);

    void matche(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch);

    void matcheSIFTCut(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch);

    void matcheSIFTCutDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                            vector<pair<unsigned, unsigned> > &vertexMatch,
                            vector<SpatialMatchInfo> &matchInfo);

    void matcheM(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch);

    void matche2(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch);

    void drawMatch(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch, Mat colorImg);

    void drawMatchOnImgs(Mat &result, const Size2i &imgSZ, SonarDescritor *sd1, Mat &img1, SonarDescritor *sd2, Mat &img2, vector<pair<unsigned, unsigned> > &vertexMatch);

    void drawVertexMatch(Mat colorImg,unsigned idU, const Gaussian& vu , const vector<GraphLink *> &eu,
                                    unsigned idV,const Gaussian &vv, const vector<GraphLink *> &ev, const Rect &drawingArea);

};

//// ===== Brutal Force explore test ====================

//#include <iostream>

//#include <cstdio>
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include "Sonar.h"
//#include "SonarConfig.h"
//#include "GroundTruth/GroundTruth.h"
//#include "GraphMatcher.h"
//#include "MatchViewer.h"

//using namespace std;
//using namespace cv;

//int main(int argc, char* argv[])
//{
//    typedef pair<unsigned,unsigned> PUU;
//    typedef pair<float,PUU> PFPUU;

//    GraphMatcher gm;

//    vector< vector< vector<PFPUU> > > explore;
//    vector<GraphLink *> u;

//    u.push_back(new GraphLink(0,0,1,1));
//    u.push_back(new GraphLink(0,0,1,2));
//    u.push_back(new GraphLink(0,0,1,3));
//    u.push_back(new GraphLink(0,0,1,4));
//    u.push_back(new GraphLink(0,0,1,5));
//    u.push_back(new GraphLink(0,0,1,6));
//    u.push_back(new GraphLink(0,0,1,7));
//    u.push_back(new GraphLink(0,0,1,8));
//    u.push_back(new GraphLink(0,0,1,9));
//    u.push_back(new GraphLink(0,0,1,10));
//    u.push_back(new GraphLink(0,0,1,11));
//    u.push_back(new GraphLink(0,0,1,12));
//    u.push_back(new GraphLink(0,0,1,13));
//    u.push_back(new GraphLink(0,0,1,14));
//    u.push_back(new GraphLink(0,0,1,15));
//    u.push_back(new GraphLink(0,0,1,16));
//    u.push_back(new GraphLink(0,0,1,17));
//    u.push_back(new GraphLink(0,0,1,18));
//    u.push_back(new GraphLink(0,0,1,19));
//    u.push_back(new GraphLink(0,0,1,20));
//    u.push_back(new GraphLink(0,0,1,21));
//    u.push_back(new GraphLink(0,0,1,22));

//    Cronometer c;
//    gm.brutalFORCExplore(u,explore);
//    float t = c.read();

//    unsigned k = 0;
//    for(unsigned i = 0 ; i < explore.size() ; i++)
//    {
//        cout << i << " , " << explore[i].size() << endl;
//        k+= explore[i].size();
//    }
//    cout << "Total , " << k << endl;
//    cout << "time , " << t << endl;

//    return 0;
//}


#endif // GRAPHMATCHER_H
