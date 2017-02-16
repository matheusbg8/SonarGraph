#ifndef GRAPHMATCHER_H
#define GRAPHMATCHER_H


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include <SonarDescritor.h>
#include <vector>
#include <list>
#include <stack>

#include "SonarConfig.h"

using namespace std;
using namespace cv;


class GraphMatcher
{
private:
    float distThreshold;
    float angThreshold;

    float stdXWeightError;
    float stdYWeightError;
    float stdIWeightError;
    float angWeightError;
    float invAngWeightError;
    float edgeDistanceWeightError;

    bool drawVertexMatching;

    static void staticCopyVertex(const vector<GraphLink*> &source, vector<GraphLink> &dest);
    static void dinamicCopyVertex(const vector<GraphLink*> &source, vector<GraphLink *> &dest);
    static void freeVertex(vector<GraphLink *> &vertex);
    static void copyVertexList(const vector<GraphLink*> &source, list<GraphLink> &dest);

    bool findPairOfVertex(vector<GraphLink *> &u, vector<GraphLink *> &v, float *invAngError, float *edgeDistError, float *angCorrectionMean, int *edgeCut);
    void compVertexDist(const vector<GraphLink *> &u, const vector<GraphLink *> &v);

    void buscaMatch(GraphLink *link, vector<GraphLink*> vertex, int star, int end, float ang);

public:
    vector< vector<GraphLink*> > graph;

    GraphMatcher(const SonarConfig& config = SonarConfig());

    void matche(SonarDescritor *sd);

    void matche(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch);
    void matche2(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch);

    void drawMatch(SonarDescritor *sd1, SonarDescritor *sd2, vector<pair<unsigned, unsigned> > &vertexMatch, Mat colorImg);

    void drawVertexMatch(Mat colorImg,unsigned idU, const Gaussian& vu , const vector<GraphLink *> &eu,
                                    unsigned idV,const Gaussian &vv, const vector<GraphLink *> &ev, const Rect &drawingArea);

};

#endif // GRAPHMATCHER_H
