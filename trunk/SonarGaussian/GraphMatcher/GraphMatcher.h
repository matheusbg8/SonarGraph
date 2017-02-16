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

#include "Sonar/SonarConfig/ConfigLoader.h"
#include "GraphMatcher/MatchInfo/MatchInfo.h"
#include "GraphMatcher/MatchInfo/MatchInfoExtended.h"
#include "GraphMatcher/MatchInfo/MatchInfoWeighted.h"

#include "GraphMatcher/VertexMatcher/VertexMatcher.h"
#include "GraphMatcher/GraphMatchFinder/GraphMatchFinder.h"

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

    static float scaleneError(float degreeAngDiff, float length1 , float length2);

    VertexMatcher *m_vm;
    GraphMatchFinder *m_gmf;

    GraphMatcher(ConfigLoader &config);
    GraphMatcher();

    void setVertexMatcher(VertexMatcher *vm);
    void setGraphFinder(GraphMatchFinder *gmf);


    void loadDefaultConfig();
    void load(ConfigLoader & config);

    void findMatch(SonarDescritor *sd1, SonarDescritor *sd2,
                   vector<MatchInfo> &vertexMatch);

    void findMatchDebug(SonarDescritor *sd1, SonarDescritor *sd2,
                        vector<MatchInfoExtended> &matchInfo);


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
