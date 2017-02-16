#ifndef CSVMATRIXGENERATOR_H
#define CSVMATRIXGENERATOR_H

#include <cstdio>
#include <vector>
#include <utility>

#include "GraphMatcher/GraphMatcher.h"
#include "Sonar/Sonar.h"
#include "GroundTruth/GroundTruth.h"
#include "GroundTruth/FrameGT.h"

using namespace std;

typedef vector<vector<unsigned> > SG;// Simple Graph
typedef pair<unsigned, SG> PUSG; // Edge with unsigned(dst) and simple graph (matchs between frames)

class CSVMatrixGenerator
{
public:
    typedef vector<vector<float> > CSVMat;
    typedef pair<unsigned, GraphLink*> CSVEdge;

    GraphMatcher gm;

    std::vector<FrameGT> m_frames;

    std::vector<vector<PUSG> > GTFrameMatchs;

    float distToLinkGraph,
          VertexBalance;

    string prefix;

public:
    CSVMatrixGenerator();

    void saveCSV(const char *fileName, const CSVMat &m);
    void loadCSV(const char *fileName, const CSVMat &m);
    void print(const CSVMat &m);

    void loadGroundTruth(const char *fileName);
    bool loadEmptyFrames(const char *fileName);

    void computeGTGaussianMatchs(SG &graph, FrameGT frame, unsigned dstFr);
    void computeGTGaussianMatchs(vector<vector<PUSG> > &graph, vector<FrameGT> &frames);

    void computeGTFrameMatchs(vector<FrameGT> &frames);

    void describeFrames(vector<FrameGT*> &frames, Sonar &sonar,
                        vector<SonarDescritor *> &descriptors);

    void generateGroundTruthCSV(FrameGT &uFr, FrameGT &vFr, SG& matchs);
    void generateGroundTruthCSV(vector<vector<PUSG> > &GTFrameMatchs);

    void generateGraphsPointsCSV(FrameGT &frame);
    void generateGraphsPointsCSV(vector<FrameGT> &frames);

    void generateGaussianVertexCSV(FrameGT &frame);
    void generateGaussianVertexCSV(vector<FrameGT> &frames);

    void generateEgCSV(unsigned frId, vector<CSVEdge> &enumEdges);

    void generateGH(unsigned frId, vector<Gaussian> &vertices, vector<CSVEdge> &edges);

    void generateVisCSV(unsigned frId, vector<Gaussian> &vertices, vector<CSVEdge> &edges);

    void generateEg(vector<vector<GraphLink*> > &g, CSVMat &Eg);
    void enumEdges(vector<vector<GraphLink*> > &g, vector<CSVEdge> &enumEdges);

    float computeSymmetricError(Gaussian &u , Gaussian &v);
    float computeSymmetricError(GraphLink *u , GraphLink *v);

    void generateKp_Kq(SonarDescritor &sdU, vector<CSVEdge> &Eu,
                       SonarDescritor &sdV, vector<CSVEdge> &Ev,
                       CSVMat &Kp, CSVMat &Kq);

    void generateCSVs(vector<FrameGT> &frames, vector<vector<PUSG> > &matchs);

    void generate(const char *groundTruthFileName);

};

#endif // CSVMATRIXGENERATOR_H
