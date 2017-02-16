#ifndef GTLOOP_H
#define GTLOOP_H

#include "GTLoopFrame.h"
#include "RoboMap/RoboMap.h"
#include "CGALDef.h"

// ==== Begin Debug Section ====
#define GTLoop_DEBUG
// ===== End Debug Section =====

class GTLoopEdge
{
public:
    unsigned srcId, dstId;
    double score;
    GTLoopEdge(unsigned srcId=0u, unsigned dstId=0u, double score=0.0):
        srcId(srcId),dstId(dstId),score(score){}
};

class GTLoop
{
public:
    GTLoop();
    ~GTLoop();

    vector<GTLoopFrame> m_frames;
    vector<GTLoopEdge> m_matchEdges;

    // Indexed Ground Truth
    vector<vector<GTLoopEdge*> >
                    m_match;

    RoboMap roboMap;
    string datasetPath;

    float maxDistBetweenCenters,
          maxDistBetweenPositions;

    void showGTLoop();

    bool loadGTLoopWithTimeStamp(const string &file);
    bool loadGTLoop(const string &file);
    void indexMatchs();

    bool createGTLoop();

    bool createGTLoopPedroOtavio();
    double sonar_match_2(double dx, double dy, double dphi, double raio);
    double sonar_match_3(Point2f &p1, double h1, Point2f &p2, double h2);

    bool saveGTLoop(const string &fileName);
    bool saveFrameInfo(const string &fileName);


    void generateHeadingDiffImage();

public:
    void start();
};

void CGAL2CV(Polygon_2 &cgalPoly, Point2f *cvPoly);
void CV2CGAL(unsigned nPts, Point2f *cvPoly, Polygon_2 &cgalPoly);


#endif // GTLOOP_H
