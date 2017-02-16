#ifndef GRAPHMATCHER_H
#define GRAPHMATCHER_H

// ===== DEBUG ===========
// #define DEBUG_GRAPHMACHER
// ==== END DEBUG ========

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include "Sonar/SonarDescritor.h"
#include <vector>
#include <list>
#include <stack>

#include "Sonar/SonarConfig.h"
#include "GraphMatcher/MatchInfoExtended.h"
#include "GraphMatcher/MatchInfoWeighted.h"
#include "GraphMatcher/EdgeMatchInfo.h"

#include "GraphMatcher/FilterMatch.h"
#include "GraphMatcher/VertexMatcher.h"


using namespace std;
using namespace cv;

class GraphMatcher
{
public:

//private:
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

    // ==== Tools ===
    FilterMatch *fm;
    VertexMatcher *gm;
    // ==============

    void computeEdgeError(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          float *angScore, float *distScore,
                          unsigned *edgeMatch, unsigned *edgeCut);

    void computeEdgeErrorEscaleno(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          float *errorScore, unsigned *edgeMatch,
                          unsigned *edgeCut);

    void computeEdgeMatch(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          vector<PUUPFF> &matchEdges);

// ====== NEW ================
    float computeEdgeMatchAxe(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          vector<PUUPFF> &matchEdges);

    float findCompativelEdgesByLenght(unsigned sourceEdgeId,
                                     vector<GraphLink *> &u, vector<GraphLink *> &v,
                                     vector<PUU> &match);

    float findCompativelEdges(unsigned uBeginEdgeId, unsigned vBeginEdgeId,
                                     vector<GraphLink *> &u, vector<GraphLink *> &v,
                                     vector<PUUPFF> &match);

    bool exploreBestEdgeMatch(float &baseError, float uAngDiff, float uEndLength,
                              unsigned vBeginId, unsigned &vEndId,int &vCount,
                              vector<GraphLink*> &v);

    float computeEdgeAngDiff(GraphLink *uBegin, GraphLink *uEnd);

    float computeScoreBetweenEdges(GraphLink *uBegin, GraphLink *uEnd,
                                   GraphLink *vBegin, GraphLink *vEnd);

    void findGraphMatchWithInitialGuess(vector<vector<GraphLink *> > &u, vector<vector<GraphLink *> > &v,
                                        MatchInfoExtended &matchEdges, vector<MatchInfoExtended> &graphMatch);
// ===========================
    void computeAxeEdgeMatch_score();

    void computeAxeEdgeMatch(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          vector<PUUPFF> &matchEdges);

    void computeEdgeMatchEscaleno(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          vector<PUUPFF> &matchEdges);

    void computeEdgeErrorGula(vector<GraphLink *> &u, vector<GraphLink *> &v,
                              float *angScore, float *distScore,
                              unsigned *edgeMatch, unsigned *edgeCut);

    bool findPairOfVertex(vector<GraphLink *> &u, vector<GraphLink *> &v,
                          float *invAngError, float *edgeDistError,
                          float *angCorrectionMean, int *edgeCut);

    void findPairOfVertexByDistance(vector<GraphLink *> &u, vector<GraphLink *> &v,
                                    float *edgeAngError, float *edgeDistError, float *angCorrectionMean,
                                    unsigned *edgeMatch, unsigned *edgeCut);

    void compVertexDist(const vector<GraphLink *> &u, const vector<GraphLink *> &v);

    void buscaMatch(GraphLink *link, vector<GraphLink*> vertex, int star, int end, float ang);

    void findVerticesMatchByAngDiff(vector<GraphLink *> &u, vector<GraphLink *> &v);


    void findBestEdgeMatch(vector<GraphLink *> &u, vector<GraphLink *> &v,
                           unsigned ui, unsigned vi,
                           unsigned &uj, unsigned &vj);

public:

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

    void matche(SonarDescritor *sd1, SonarDescritor *sd2,
                vector<pair<unsigned, unsigned> > &vertexMatch);

    void matcheSIFTCut(SonarDescritor *sd1, SonarDescritor *sd2,
                       vector<MatchInfo> &vertexMatch);

    void matcheScalene(SonarDescritor *sd1, SonarDescritor *sd2,
                       vector<MatchInfoWeighted> &vertexMatch);

    void SiftCut(vector<MatchInfoWeighted> &vertexMatch,
                 unsigned guSize, unsigned gvSize);

    void matcheSIFTCutDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                            vector<pair<unsigned, unsigned> > &vertexMatch,
                            vector<MatchInfoExtended> &matchInfo);

    void computeCompleteMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                              vector<vector<MatchInfoExtended> > &edgeInfo);


    void NEWMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                  vector<pair<unsigned, unsigned> > &vertexMatch,
                  vector<MatchInfoExtended> &matchInfo);

    void matcheM(SonarDescritor *sd1, SonarDescritor *sd2,
                 vector<MatchInfo> &vertexMatch);

    void matche2(SonarDescritor *sd1, SonarDescritor *sd2,
                 vector<MatchInfo> &vertexMatch);

    void drawMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                   vector<MatchInfo> &vertexMatch,
                   Mat colorImg);

    void drawMatchOnImgs(Mat &result, const Size2i &imgSZ,
                         SonarDescritor *sd1, Mat &img1,
                         SonarDescritor *sd2, Mat &img2,
                         vector<MatchInfo> &vertexMatch);

    void drawVertexMatch(Mat colorImg,unsigned idU, const Gaussian& vu , const vector<GraphLink *> &eu,
                                  unsigned idV,const Gaussian &vv, const vector<GraphLink *> &ev, const Rect &drawingArea);

};

#endif // GRAPHMATCHER_H
