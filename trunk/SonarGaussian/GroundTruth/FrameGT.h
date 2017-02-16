#ifndef FRAMEGT_H
#define FRAMEGT_H

#include <string>
#include <vector>

#include "Sonar/Gaussian.h"
#include "WindowTool/Frame.h"

using namespace std;

// ==== DEBUG Section ====
//#define FRAME_TRACKING_DEBUG

// === end debug section ==


/**
 * @brief This is a modifficated Frame
 * used by WFGraoundTruth to store
 * informations about ground truth matchs.
 *
 */
class FrameGT: public Frame
{
public:
    vector<Gaussian> gaussians;
    unsigned threshold1, threshold2;

    typedef pair<unsigned,unsigned> PUU;
    //                [srcGaussian][MatchId]<FrameID , dest Gaussian ID>
    vector< vector<PUU> > match;  /**< @todo - This struct need to be otimized to allow faster erasers. */

    FrameGT(const string& fileName=string(""),unsigned frameNumber=0);
    ~FrameGT();

    const Gaussian *findClosestGaussian(int x, int y, int *id);
    Gaussian *findIntersectGaussian(Gaussian &g, int *id);
    Gaussian *findGaussianPrc(float x, float y, int *id, float prec);

    void newMatch(unsigned srcGaussianID, unsigned destFrame, unsigned dstGaussianID);

    bool findMatch(unsigned u, unsigned destFrame, int *v);
    unsigned nMatchs(unsigned destFrame);

    void removeGaussianMatch(unsigned srcGauss);
    void removeGaussianMatchToFrame(unsigned srcGauss, unsigned destFrameID);
    void removeMatchToFrame(unsigned dstFrameID);
    void removeAllMatch();
    void clear();
};

ostream & operator << (ostream &os, FrameGT &fr);


#endif // FRAMEGT_H
